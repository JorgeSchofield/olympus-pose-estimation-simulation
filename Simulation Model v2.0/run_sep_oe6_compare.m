function T = run_sep_oe6_compare(N)
%RUN_SEP_OE6_COMPARE  Compara el modelo con los ensayos OE6 del rover real.
%
%   T = RUN_SEP_OE6_COMPARE(N)   N = corridas por ensayo y nivel (30 por defecto)
%
%   Rover Olympus - TFG Ingenieria Electronica ITCR / SETEC.
%
%   Los ensayos del rover no tienen grabacion de llcmux, asi que no se
%   pueden reproducir con hlc_replay. Se comparan los RESULTADOS de los
%   mismos ensayos (V-OE6-4, "el escenario simulado contra el rover"). La
%   equivalencia del estimador ya esta probada aparte (V-U-10): la
%   diferencia entre modelo y rover viene de la planta y los sensores.
%
%   Dos niveles:
%     ideal     parametros del modelo, sin defectos anadidos. Lo que el
%               estimador logra en un rover sano.
%     ajustado  rover como el real: lentitud, deriva a la izquierda, escala
%               del giroscopio y escala de distancia.
%
%   Para no razonar en circulo, el nivel ajustado se fija SOLO con la recta
%   de 5 m y los dos giros. El UMBmark no entra en el ajuste: es una
%   prediccion. La recta y los giros salen parecidos por construccion.
%
%   El error se calcula donde se midio con cinta: al centro del frente del
%   chasis, relativo a su posicion inicial.
%
%   Solo corre el bucle de MATLAB (mismas funciones de paso que Simulink;
%   run_sep_full_demo comprueba que coinciden). Usa parfor si hay Parallel
%   Computing Toolbox.
%
%   Ejecutar:  >> T = run_sep_oe6_compare(30)

    if nargin < 1, N = 30; end

    p = rover_params();

    % --- resultados del rover (agents/DESIGN.md 16.4) --------------------
    % Error [cm] o [deg] segun el ensayo.
    rover = struct('recta', 10.8, 'giroCW', 9.8, 'giroCCW', 6.07, ...
                   'umbCW', 11.7, 'umbCCW', 11.5);

    % --- geometria real de cada lado, de la cinta [m] --------------------
    % El lado 4 horario quedo 0.35 m corto. Son tres esquinas: el rover
    % termina el cuarto lado sin girar.
    lados_cw  = [1.93 1.985 2.01 1.65];
    lados_ccw = [2.00 1.999 1.80 2.01];

    % --- ajuste: SOLO recta de 5 m y los dos giros -----------------------
    hy   = p.plant.half_track(:).';
    chi  = p.plant.chi_true;
    Hc   = sum(hy) / (2*sum(hy.^2));          % giro por unidad de avance y de s
    kap  = deg2rad(13.1) / 5.046;             % [rad/m] deriva izquierda, recta 5 m
    c    = kap * chi / Hc;
    sL   = c / (1 + c/2);                     % deslizamiento de las ruedas izquierdas
    sf   = (350.2 + 353.93)/720 - 1;          % giroscopio lee bajo en los dos sentidos
    kdis = 0.997 * (1 - sL/2);                % recta: el estimador lee 0.3 % corto

    aj = struct('sL', sL, 'kap', kap, 'sf', sf, 'kdis', kdis, ...
                'v', 0.0165, 'wg', deg2rad(1.42));   % velocidades medias reales
    id = struct('sL', 0, 'kap', 0, 'sf', 0, 'kdis', 1, ...
                'v', p.drive.v_max, 'wg', p.drive.w_turn/chi);

    ensayos = {'recta', 'recta', 0,  5.0; ...
               'giroCW', 'giro', -1, 0;   ...
               'giroCCW', 'giro', +1, 0;  ...
               'umbCW', 'umbmark', -1, 0; ...
               'umbCCW', 'umbmark', +1, 0};
    lados_ideal = [2 2 2 2];

    % --- lista de corridas -----------------------------------------------
    ne = size(ensayos, 1);
    tareas = zeros(2*ne*N, 3);
    n = 0;
    for nivel = 1:2
        for e = 1:ne
            for s = 1:N
                n = n + 1;
                tareas(n,:) = [nivel e s];
            end
        end
    end

    res = zeros(n, 1);
    pos = zeros(n, 4);   % frente al final: [Xe Ye Xv Yv], X derecha, Y adelante [m]
    fprintf('OE6 modelo contra rover: %d corridas (%d ensayos, 2 niveles, N = %d)\n', n, ne, N);
    t0 = tic;
    parfor i = 1:n
        nivel = tareas(i,1);  e = tareas(i,2);  s = tareas(i,3);
        tipo = ensayos{e,2};  signo = ensayos{e,3};
        if nivel == 1
            cfg = id;  lados = lados_ideal;
        else
            cfg = aj;
            if signo < 0, lados = lados_cw; else, lados = lados_ccw; end
        end
        if strcmp(tipo, 'recta'), lados = ensayos{e,4}; end
        [perr, herr, fin] = una_corrida(p, cfg, tipo, signo, lados, p.sim.seed + 1000*e + s);
        if strcmp(tipo, 'giro'), res(i) = herr; else, res(i) = perr; end
        pos(i,:) = fin;
    end
    fprintf('Terminado en %.1f min\n\n', toc(t0)/60);

    % --- tabla -----------------------------------------------------------
    nombre = {'Recta 5 m [cm]'; 'Giro 360 CW [deg]'; 'Giro 360 CCW [deg]'; ...
              'UMBmark CW [cm]'; 'UMBmark CCW [cm]'};
    papel  = {'ajuste'; 'ajuste'; 'ajuste'; 'prediccion'; 'prediccion'};
    Rover = zeros(ne,1);  MedI = Rover;  SdI = Rover;  MedA = Rover;  SdA = Rover;  Z = Rover;
    for e = 1:ne
        Rover(e) = rover.(ensayos{e,1});
        rI = res(tareas(:,1) == 1 & tareas(:,2) == e);
        rA = res(tareas(:,1) == 2 & tareas(:,2) == e);
        MedI(e) = mean(rI);  SdI(e) = std(rI);
        MedA(e) = mean(rA);  SdA(e) = std(rA);
        Z(e) = (Rover(e) - MedA(e)) / SdA(e);
    end
    T = table(nombre, Rover, MedI, SdI, MedA, SdA, Z, papel, ...
              'VariableNames', {'Ensayo','Rover','IdealMedia','IdealSD', ...
                                'AjustadoMedia','AjustadoSD','Z','Papel'});

    fprintf('Ajuste: deslizamiento izq %.2f %%, deriva %.2f deg/m, escala giroscopio %+.2f %%, escala distancia %.4f\n', ...
            100*sL, rad2deg(kap), 100*sf, kdis);
    fprintf('\n%-20s %8s %16s %18s %7s  %s\n', 'Ensayo', 'Rover', 'Ideal', 'Ajustado', 'Z', 'Papel');
    for e = 1:ne
        dentro = 'dentro de 2 SD';
        if abs(Z(e)) > 2, dentro = 'FUERA de 2 SD'; end
        fprintf('%-20s %8.2f %8.2f +- %5.2f %9.2f +- %5.2f %7.2f  %s, %s\n', nombre{e}, Rover(e), ...
                MedI(e), SdI(e), MedA(e), SdA(e), Z(e), papel{e}, dentro);
    end

    % --- diferencia vectorial (revision del borrador, item 2.7) ---------
    % Vector de error = posicion estimada menos verdadera del frente, al
    % final. Se compara el del rover (cinta) con la media del modelo.
    roverE = struct('recta', [-0.803 4.969; -0.70 5.00], ...
                    'umbCW', [0.295 -0.206; 0.275 -0.322], ...
                    'umbCCW', [0.009 0.081; -0.047 -0.020]);   % [estimado; cinta]
    fprintf('\nDiferencia vectorial del error final (cm): rover frente a la media del modelo\n');
    for e = [1 4 5]
        k = ensayos{e,1};
        er = 100*(roverE.(k)(1,:) - roverE.(k)(2,:));
        for nivel = 1:2
            sel = tareas(:,1) == nivel & tareas(:,2) == e;
            em = 100*mean(pos(sel,1:2) - pos(sel,3:4), 1);
            sd = 100*std(pos(sel,1:2) - pos(sel,3:4), 0, 1);
            fv = mean(pos(sel,3:4), 1);
            fprintf('%-8s %-8s rover (%+6.1f, %+6.1f)  modelo (%+6.1f +- %4.1f, %+6.1f +- %4.1f)  |dif| %5.1f cm; final verdadero modelo (%.3f, %.3f) m\n', ...
                    k, ifelse(nivel == 1, 'ideal', 'ajustado'), er(1), er(2), em(1), sd(1), em(2), sd(2), ...
                    norm(er - em), fv(1), fv(2));
        end
    end
    save(fullfile(fileparts(mfilename('fullpath')), 'oe6_compare_result.mat'), ...
         'T', 'res', 'pos', 'tareas', 'ensayos', 'aj', 'id', 'N');
end

function s = ifelse(c, a, b)
    if c, s = a; else, s = b; end
end

function [perr_cm, herr_deg, fin] = una_corrida(p, cfg, tipo, signo, lados, semilla)
%UNA_CORRIDA  Un ensayo con una semilla. Devuelve el error final en el frente.
    Tb  = p.sim.Tb;
    tp  = p.sim.pause_s;
    chi = p.plant.chi_true;
    Lf  = 0.1615;                       % [m] centro del chasis al frente

    v_cmd = cfg.v / (1 - cfg.sL/2);     % el encoder ve el arco, el suelo avanza menos
    w_cmd = cfg.wg * chi;

    % --- perfil --------------------------------------------------------
    t = 10;                             % reposo inicial: el rover real paraba mas
    seg = [0 0 0];
    switch tipo
        case 'recta'
            [seg, t] = tramo(seg, t, v_cmd, 0, lados/cfg.v, tp);
        case 'giro'
            [seg, t] = tramo(seg, t, 0, signo*w_cmd, 2*pi/cfg.wg, tp);
        case 'umbmark'
            for i = 1:numel(lados)
                [seg, t] = tramo(seg, t, v_cmd, 0, lados(i)/cfg.v, tp);
                if i < numel(lados)
                    % El operador gira HASTA alinear el rover con el lado
                    % siguiente, asi que descuenta la deriva del lado recorrido.
                    ang = signo*pi/2 - cfg.kap*lados(i);
                    [seg, t] = tramo(seg, t, 0, sign(ang)*w_cmd, abs(ang)/cfg.wg, tp);
                end
            end
    end

    Nk = round(t/Tb);
    tt = (0:Nk-1).'*Tb;
    cmd = zeros(Nk,2);
    for k = 1:size(seg,1)
        cmd(tt >= seg(k,1), 1) = seg(k,2);
        cmd(tt >= seg(k,1), 2) = seg(k,3);
    end

    tr = p.plant.terrain;
    pitch = tr.pitch_dc + tr.pitch_amp*sin(2*pi*tr.pitch_f*tt);
    roll  = tr.roll_dc  + tr.roll_amp *sin(2*pi*tr.roll_f *tt + 1.1);

    % Deslizan las ruedas izquierdas (FL CL RL) solo al avanzar: los encoders
    % de los dos lados cuentan parecido y aun asi el rover se va a la izquierda.
    slip = zeros(Nk,6);
    slip(cmd(:,1) ~= 0, [2 4 6]) = cfg.sL;

    rng(semilla);
    noise = randn(Nk,5);
    uch   = rand(Nk,3);

    % --- defectos: el filtro cree la escala nominal, la planta no -------
    imu = p.imu;
    imu.gyro_sf_err = cfg.sf;
    geo_hlc = p.geo;
    geo_hlc.m_per_tick_R = p.geo.m_per_tick_R * cfg.kdis;
    geo_hlc.m_per_tick_L = p.geo.m_per_tick_L * cfg.kdis;
    geo_hlc.B_eff        = p.geo.B_eff * cfg.kdis;

    stP = plant_init();
    stL = llc_init();
    stC = channel_init(p.chan);
    stH = hlc_init(p.ekf);

    tru = [0 0 0];  est = [0 0 0];
    for k = 1:Nk
        [stP, o] = plant_step(stP, cmd(k,1), cmd(k,2), slip(k,:), ...
                              pitch(k), roll(k), Tb, p.plant);
        [stL, b, nb, em] = llc_step(stL, o.w, o.a_body, pitch(k), roll(k), ...
                                    o.arc, noise(k,:), p.llc, p.geo, imu, Tb);
        [stC, cb, cn, ar] = channel_step(stC, b, nb, em, uch(k,:), p.chan, Tb);
        [stH, x, ~, ~, ~, pb] = hlc_step(stH, cb, cn, ar, ...
                                         geo_hlc, p.ekf, p.agents, Tb, uch(k,3));
        if pb
            est = x(1:3).';
            tru = o.pose;
        end
    end

    fe = [est(1) + Lf*cos(est(3)), est(2) + Lf*sin(est(3))];
    ft = [tru(1) + Lf*cos(tru(3)), tru(2) + Lf*sin(tru(3))];
    perr_cm  = 100 * hypot(fe(1)-ft(1), fe(2)-ft(2));
    herr_deg = rad2deg(abs(mod(est(3)-tru(3) + pi, 2*pi) - pi));
    % marco de la cinta: X a la derecha, Y adelante, relativo al frente inicial
    fin = [-fe(2), fe(1) - Lf, -ft(2), ft(1) - Lf];
end

function [seg, t] = tramo(seg, t, v, w, dur, tp)
%TRAMO  Anade un movimiento de duracion dur seguido de una pausa tp.
    seg(end+1,:) = [t v w];
    t = t + dur;
    seg(end+1,:) = [t 0 0];
    t = t + tp;
end
