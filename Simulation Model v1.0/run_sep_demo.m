%% RUN_SEP_DEMO  Modelo de referencia del Subsistema de Estimacion de Pose.
%
%  Rover Olympus - TFG Ingenieria Electronica ITCR / SETEC.
%  Ref.: DRT-SEP-001 v0.1.
%
%  Que hace:
%   1) Simula la VERDAD de terreno del rover sobre un recorrido UMBmark
%      (cuadrado en ambos sentidos) o una recta o un giro en el sitio.
%   2) Sintetiza las cuentas ACUMULADAS de los seis encoders con radios
%      desiguales (error sistematico Ed), ancho de via efectivo real
%      (error Eb) y eventos de deslizamiento, mas el giroscopio con sesgo
%      y ruido.
%   3) Construye y ejecuta el modelo Simulink, y corre ademas el mismo
%      filtro en MATLAB puro como verificacion cruzada.
%   4) Evalua el indicador PE-RNF-004: error final como porcentaje de la
%      distancia recorrida.
%
%  El GPS se genera y se dibuja, pero NUNCA entra al filtro: es verdad de
%  terreno para validacion offline (PE-CON-001). El grafico muestra, de
%  paso, por que a la escala de este ensayo no sirve como referencia.
%
%  Requisitos en la carpeta: rover_params.m, sep_geo_params.m,
%  sep_ekf_params.m, sep_odometry.m, sep_ekf_step.m, build_sep_model.m
%
%  Ejecutar:  >> run_sep_demo

clear; clc; close all;

p = rover_params();               % imprime la verificacion de consistencia
rng(p.sim.seed);

Ts = p.time.T_ekf;
v_nom = p.drive.v_max;
if strcmpi(p.drive.hypothesis, 'B')
    v_nom = v_nom * 10;
    fprintf('Hipotesis B activa: v_max escalada a %.3f m/s\n', v_nom);
end

%% ============ 1. VERDAD DE TERRENO ============
[cmd, dirLabel] = build_scenario(p, v_nom);

results = struct('dir', {}, 'err_pct', {}, 'err_final', {}, 'dist', {});

for run_i = 1:numel(cmd)
    seg = cmd{run_i};

    % --- integracion de la verdad + sintesis de sensores ---
    [TRU, enc_counts, wz, dist_true, gps] = simulate_truth(seg, p, Ts, v_nom);
    N = size(TRU,1);
    t = (0:N-1).'*Ts;

    % Incrementos de cuentas: el HLC diferencia las cuentas ACUMULADAS que
    % llegan por el Canal 2. Diferenciar aqui, no cuantizar incrementos.
    enc_delta = [enc_counts(1,:); diff(enc_counts,1,1)];

    enc_log  = [t, enc_delta];     %#ok<NASGU>
    gyro_log = [t, wz];            %#ok<NASGU>

    %% ============ 2. SIMULINK ============
    modelName = 'sep_ekf_rover';
    useSimulink = true;
    EST_sl = [];
    try
        build_sep_model(modelName, Ts);
        set_param(modelName, 'StopTime', num2str(t(end)));
        simOut = sim(modelName);
        xlog   = simOut.get('x_est_log');
        EST_sl = squeeze(xlog.Data).';
        if size(EST_sl,1) ~= N
            EST_sl = interp1(xlog.Time, EST_sl, t);
        end
    catch ME
        useSimulink = false;
        warning(['El modelo Simulink no se ejecuto (%s). Se usa el bucle MATLAB. ' ...
                 'Para ver la causa real, ejecutar sim(''%s'') sin try/catch.'], ...
                ME.message, modelName);
    end

    %% ============ 3. VERIFICACION CRUZADA EN MATLAB PURO ============
    geo = sep_geo_params();
    prm = sep_ekf_params();
    x = zeros(5,1);
    P = diag(prm.P0);
    EST  = zeros(N,5);
    DIAG = zeros(N,3);
    for k = 1:N
        [ds, dth, moving, ~] = sep_odometry(enc_delta(k,:).', geo);
        [x, P, ~, d] = sep_ekf_step(x, P, ds, dth, wz(k), Ts, moving, prm);
        EST(k,:)  = x.';
        DIAG(k,:) = d.';
    end

    %% ============ 4. METRICAS (PE-RNF-004) ============
    perr = hypot(EST(:,1)-TRU(:,1), EST(:,2)-TRU(:,2));
    herr = abs(wrapPiLocal(EST(:,3)-TRU(:,3)));
    err_pct = 100*perr(end)/dist_true;

    fprintf('\n---------- %s ----------\n', dirLabel{run_i});
    fprintf('  distancia recorrida ......... %.2f m\n', dist_true);
    fprintf('  duracion .................... %.1f s (%.1f min)\n', t(end), t(end)/60);
    fprintf('  error final de posicion ..... %.2f cm\n', 100*perr(end));
    fprintf('  ERROR / DISTANCIA ........... %.2f %%   (PE-RNF-004: <= 3 %%)\n', err_pct);
    fprintf('  error maximo ................ %.2f cm\n', 100*max(perr));
    fprintf('  error de rumbo final ........ %.3f deg\n', rad2deg(herr(end)));
    fprintf('  sesgo giro est / verdadero .. %.5f / %.5f rad/s\n', EST(end,5), p.imu.gyro_bias_z);
    fprintf('  pasos con deslizamiento ..... %d de %d\n', sum(DIAG(:,2)>1.01), N);
    if useSimulink
        fprintf('  max |Simulink - MATLAB| ..... %.2e  (deberia ser ~0)\n', ...
                max(abs(EST(:)-EST_sl(:))));
    end
    if err_pct <= 3.0
        fprintf('  >> CUMPLE el indicador con los parametros actuales.\n');
    else
        fprintf('  >> NO CUMPLE. Revisar calibracion antes que sintonizacion.\n');
    end

    results(run_i).dir       = dirLabel{run_i};
    results(run_i).err_pct   = err_pct;
    results(run_i).err_final = perr(end);
    results(run_i).dist      = dist_true;

    %% ============ 5. GRAFICAS ============
    figure('Color','w','Position',[80 80 1150 720]);

    subplot(2,3,[1 4]); hold on; grid on; axis equal
    plot(TRU(:,1), TRU(:,2), 'k-',  'LineWidth', 2,   'DisplayName','verdad');
    plot(EST(:,1), EST(:,2), 'r--', 'LineWidth', 1.3, 'DisplayName','EKF');
    if p.sim.gps_on
        plot(gps(:,1), gps(:,2), 'c.', 'MarkerSize', 8, ...
             'DisplayName', sprintf('GPS (CEP %.1f m, NO entra al filtro)', p.sim.gps_cep));
    end
    plot(TRU(1,1), TRU(1,2), 'ko', 'MarkerFaceColor','g', 'DisplayName','inicio');
    plot(EST(end,1), EST(end,2), 'rs','MarkerFaceColor','r','DisplayName','fin estimado');
    title(sprintf('Trayectoria - %s', dirLabel{run_i}));
    xlabel('x [m]'); ylabel('y [m]'); legend('Location','best');

    subplot(2,3,2); plot(t, 100*perr, 'b'); grid on
    title('Error de posicion'); xlabel('t [s]'); ylabel('[cm]');

    subplot(2,3,3); plot(t, rad2deg(herr), 'r'); grid on
    title('Error de rumbo'); xlabel('t [s]'); ylabel('[deg]');

    subplot(2,3,5); hold on; grid on
    plot(t, EST(:,5), 'b', 'DisplayName','b_\omega estimado');
    yline(p.imu.gyro_bias_z, 'k:', 'DisplayName','verdadero');
    title('Sesgo del giroscopio (se fija por ZARU en reposo)');
    xlabel('t [s]'); ylabel('[rad/s]'); legend('Location','best');

    subplot(2,3,6); hold on; grid on
    yyaxis left;  plot(t, DIAG(:,1)); ylabel('|discrepancia| [rad/s]');
    yyaxis right; plot(t, DIAG(:,2)); ylabel('ganancia de Q');
    yline(prm.slip_thresh, 'k:');
    title('Deteccion de deslizamiento'); xlabel('t [s]');

    sgtitle(sprintf('SEP - EKF odometria 6 ruedas + giroscopio | %s', dirLabel{run_i}));
end

%% ============ 6. RESUMEN UMBmark ============
if numel(results) > 1
    fprintf('\n================ RESUMEN UMBmark ================\n');
    for i = 1:numel(results)
        fprintf('  %-22s %6.2f cm  (%.2f %% de %.1f m)\n', ...
                results(i).dir, 100*results(i).err_final, ...
                results(i).err_pct, results(i).dist);
    end
    fprintf('  El contraste entre sentidos separa el error de diametros\n');
    fprintf('  desiguales (Ed) del error de ancho de via (Eb).\n');
    fprintf('=================================================\n');
end


%% ===================== FUNCIONES LOCALES =====================

function [cmd, labels] = build_scenario(p, v_nom)
%BUILD_SCENARIO  Devuelve la lista de segmentos {tipo, duracion} por corrida.
    tp = p.sim.pause_s;
    switch lower(p.sim.scenario)
        case 'recta'
            L = p.sim.side*2;
            cmd    = { {'p',tp; 's',L/v_nom; 'p',tp} };
            labels = {'Recta'};
        case 'giro'
            cmd    = { {'p',tp; 't',2*pi/p.sim.w_turn; 'p',tp} };
            labels = {'Giro en el sitio'};
        otherwise    % umbmark
            t_side = p.sim.side/v_nom;
            t_turn = (pi/2)/p.sim.w_turn;
            cw = {'p',tp};  ccw = {'p',tp};
            for i = 1:4
                cw  = [cw;  {'s',t_side}; {'t',-t_turn}]; %#ok<AGROW>
                ccw = [ccw; {'s',t_side}; {'t', t_turn}]; %#ok<AGROW>
            end
            cw  = [cw;  {'p',tp}];
            ccw = [ccw; {'p',tp}];
            cmd    = {cw, ccw};
            labels = {'UMBmark horario', 'UMBmark antihorario'};
    end
end

% ---------------------------------------------------------------------
function [TRU, enc_counts, wz, dist, gps] = simulate_truth(seg, p, Ts, v_nom)
%SIMULATE_TRUTH  Integra la verdad y sintetiza encoders y giroscopio.
%
%  Los encoders devuelven cuentas ACUMULADAS, igual que el Canal 2 real.
%  Se acumula el arco de cada rueda en metros y se trunca a ticks; asi el
%  residuo sub-tick se arrastra de un paso al siguiente, que es como se
%  comporta un encoder fisico. Cuantizar cada incremento por separado
%  introduciria un sesgo determinista de redondeo.

    geo_t.R_wheel = p.geo.R_wheel .* (1 + 0.01*randn(1,6));   % dispersion -> Ed
    geo_t.B_eff   = p.sim.chi_true * p.geo.B_nom;             % ancho real -> Eb
    mpt_true      = 2*pi*geo_t.R_wheel / p.geo.ticks_per_rev;

    px = 0; py = 0; th = 0; dist = 0;
    arc   = zeros(1,6);
    cnt   = zeros(1,6);
    TRU = []; ENC = []; WZ = []; GPS = [];
    k_gps = round(1/(p.sim.gps_rate*Ts));
    k = 0;

    for i = 1:size(seg,1)
        kind = seg{i,1};
        n    = max(1, round(abs(seg{i,2})/Ts));
        for j = 1:n
            k = k + 1;
            switch kind
                case 's'                                  % recta
                    v = v_nom;  w = 0;
                case 't'                                  % giro en el sitio
                    v = 0;      w = sign(seg{i,2})*p.sim.w_turn;
                otherwise                                 % pausa (habilita ZARU)
                    v = 0;      w = 0;
            end

            % --- deslizamiento: las ruedas giran, el rover no avanza ---
            sl = 0;
            if strcmpi(kind,'s') && rand < p.sim.slip_prob
                sl = p.sim.slip_frac;
            end

            ds_cmd  = v*Ts;                 % lo que "ven" los encoders
            dth_cmd = w*Ts;
            ds_true  = ds_cmd*(1-sl);       % lo que realmente avanza
            dth_true = dth_cmd*(1-0.3*sl);

            % --- verdad ---
            px = px + ds_true*cos(th + 0.5*dth_true);
            py = py + ds_true*sin(th + 0.5*dth_true);
            th = wrapPiLocal(th + dth_true);
            dist = dist + abs(ds_true);
            TRU(k,:) = [px, py, th]; %#ok<AGROW>

            % --- encoders: arco por rueda, cuentas acumuladas ---
            aR = ds_cmd + 0.5*dth_cmd*geo_t.B_eff;
            aL = ds_cmd - 0.5*dth_cmd*geo_t.B_eff;
            arc = arc + [aR aL aR aL aR aL];       % FR FL CR CL RR RL
            cnt = floor(arc ./ mpt_true);
            ENC(k,:) = cnt; %#ok<AGROW>

            % --- giroscopio ---
            WZ(k,1) = w*(1+p.imu.gyro_sf_err) + p.imu.gyro_bias_z + ...
                      deg2rad(p.imu.gyro_arw)/sqrt(Ts)*randn; %#ok<AGROW>

            % --- GPS: solo se registra, nunca entra al filtro ---
            if p.sim.gps_on && mod(k-1, k_gps) == 0
                GPS(end+1,:) = [px, py] + p.sim.gps_cep*randn(1,2)/2; %#ok<AGROW>
            end
        end
    end
    enc_counts = ENC; wz = WZ; gps = GPS;
end

% ---------------------------------------------------------------------
function a = wrapPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end
