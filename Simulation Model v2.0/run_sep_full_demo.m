%% RUN_SEP_FULL_DEMO  Modelo completo del rover: LLC + enlace + HLC.
%
%  Rover Olympus - TFG Ingenieria Electronica ITCR / SETEC.
%
%  Genera las senales de entrada, construye y ejecuta el modelo Simulink
%  completo, y corre EN PARALELO el mismo modelo como bucle de MATLAB sobre
%  las MISMAS funciones de paso base y las MISMAS secuencias de ruido.
%
%  La comparacion max|Simulink - MATLAB| debe dar ~0. Es la prueba de que
%  las dos rutas ejecutan el mismo algoritmo: dos implementaciones que
%  coinciden son evidencia, una sola es solo codigo.
%
%  Para el contraste que justifica el cambio de firmware:
%      p.llc.clock_mode = 'software';   % LLC de hoy
%      p.llc.clock_mode = 'timer';      % LLC propuesto
%
%  Ejecutar:  >> run_sep_full_demo

clear; clc; close all;

p = rover_params();
rng(p.sim.seed);
Tb = p.sim.Tb;

%% ============ 1. PERFIL DE COMANDO ============
% UMBmark COMPLETO: el cuadrado se recorre en los DOS sentidos. No es
% redundancia. Los dos errores sistematicos de Borenstein y Feng se separan
% justamente por el contraste entre sentidos: el de diametros desiguales
% (Ed) cambia de signo al invertir el recorrido y el de ancho de via (Eb)
% no. Con un solo sentido ambos quedan sumados y son indistinguibles.
%
% Las pausas NO son cortesia: son lo unico que hace observable el sesgo del
% giroscopio via ZARU. Un recorrido sin paradas lo deja sin anclar.
v = p.drive.v_max;  w = p.drive.w_turn;  tp = p.sim.pause_s;
t_side = p.sim.side/v;  t_turn = (pi/2)/w;

sentidos = {'horario', 'antihorario'};
signos   = [-1, +1];
RES = struct('dir',{},'err_pct',{},'dist',{},'her',{},'lat',{},'rate',{});

for run_i = 1:numel(signos)
seg = [0 0 0];  t = tp;
for i = 1:4
    seg(end+1,:) = [t v 0];              t = t + t_side; %#ok<SAGROW>
    seg(end+1,:) = [t 0 0];              t = t + tp;     %#ok<SAGROW>
    seg(end+1,:) = [t 0 signos(run_i)*w]; t = t + t_turn; %#ok<SAGROW>
    seg(end+1,:) = [t 0 0];              t = t + tp;     %#ok<SAGROW>
end
T_end = t;
N  = round(T_end/Tb);
tt = (0:N-1).'*Tb;

cmd = zeros(N,2);
for k = 1:N
    j = find(seg(:,1) <= tt(k), 1, 'last');
    cmd(k,:) = seg(j,2:3);
end

%% ============ 2. TERRENO, DESLIZAMIENTO Y RUIDO ============
tr = p.plant.terrain;
pitch = tr.pitch_dc + tr.pitch_amp*sin(2*pi*tr.pitch_f*tt);
roll  = tr.roll_dc  + tr.roll_amp *sin(2*pi*tr.roll_f *tt + 1.1);

slip = zeros(N,6);
sp = p.plant.slip;
slip = slip + sp.common_frac * inwin(tt, sp.common_win);
slip(:,[1 3 5]) = slip(:,[1 3 5]) + sp.side_frac * inwin(tt, sp.side_win);
slip(:,sp.wheel_idx) = slip(:,sp.wheel_idx) + sp.wheel_frac * inwin(tt, sp.wheel_win);
slip = min(slip, 0.99);

% Las secuencias de azar se generan AQUI y entran al modelo como senal.
% Es lo que hace identicas las dos rutas.
noise = randn(N,5);
uch   = rand(N,3);

cmd_log   = [tt cmd];              %#ok<NASGU>
terr_log  = [tt pitch roll slip];  %#ok<NASGU>
noise_log = [tt noise];            %#ok<NASGU>
chan_log  = [tt uch];              %#ok<NASGU>

%% ============ 3. BUCLE DE MATLAB (referencia) ============
stP = plant_init();
stL = llc_init();
stC = channel_init(p.chan);
stH = hlc_init(p.ekf);

TRU = zeros(N,3);
XM  = zeros(N,5);
LAT = nan(N,1);
PUB = false(N,1);

for k = 1:N
    [stP, o] = plant_step(stP, cmd(k,1), cmd(k,2), slip(k,:), ...
                          pitch(k), roll(k), Tb, p.plant);
    TRU(k,:) = o.pose;

    [stL, b, nb, em] = llc_step(stL, o.w, o.a_body, pitch(k), roll(k), ...
                                o.arc, noise(k,:), p.llc, p.geo, p.imu, Tb);
    [stC, cb, cn, ar] = channel_step(stC, b, nb, em, uch(k,:), p.chan, Tb);
    [stH, x, ~, ~, lt, pb] = hlc_step(stH, cb, cn, ar, ...
                                      p.geo, p.ekf, p.agents, Tb, uch(k,3));
    XM(k,:) = x.';
    PUB(k)  = pb;
    if pb, LAT(k) = lt; end
end

%% ============ 4. SIMULINK ============
modelName = 'sep_rover_full';
useSL = true;  XS = [];
try
    build_sep_full_model(modelName, Tb);
    set_param(modelName, 'StopTime', num2str(tt(end)));
    so = sim(modelName);
    XS = squeeze(so.get('pose_log').Data).';
    if size(XS,1) ~= N, XS = interp1(so.get('pose_log').Time, XS, tt); end
catch ME
    useSL = false;
    warning(['El modelo Simulink no se ejecuto (%s). Se usan los ' ...
             'resultados del bucle de MATLAB. Para ver la causa real, ' ...
             'ejecutar sim(''%s'') sin try/catch.'], ME.message, modelName);
end

%% ============ 5. METRICAS ============
idx  = find(PUB);
perr = hypot(XM(idx,1)-TRU(idx,1), XM(idx,2)-TRU(idx,2));
herr = abs(wrapPiLocal(XM(idx,3)-TRU(idx,3)));
dist = stP.dist;
err_pct = 100*perr(end)/dist;
lat = LAT(idx);
rate = numel(idx)/tt(end);
deliv = stH.n_updates / max(stC.n_sent,1);

fprintf('\n========== MODELO COMPLETO: LLC + enlace + HLC ==========\n');
fprintf('  configuracion del LLC ....... reloj %s, TX %s\n', ...
        p.llc.clock_mode, p.llc.tx_mode);
fprintf('  EXACTITUD\n');
fprintf('    distancia recorrida ....... %.2f m\n', dist);
fprintf('    error final ............... %.2f cm = %.2f %%  (req <= %.0f %%)  %s\n', ...
        100*perr(end), err_pct, p.req.err_max_pct, ok(err_pct <= p.req.err_max_pct));
fprintf('    error de rumbo final ...... %.2f deg\n', rad2deg(herr(end)));
fprintf('    sesgo giro est/verdadero .. %.5f / %.5f rad/s\n', ...
        XM(idx(end),5), p.imu.gyro_bias_z);
fprintf('  TEMPORIZACION DEL LLC\n');
fprintf('    ciclos ejecutados ......... %d en %.1f s\n', stL.k, tt(end));
fprintf('    ciclo real medio .......... %.2f ms  (el firmware dice %.0f ms)\n', ...
        1000*tt(end)/max(stL.k,1), p.llc.LOOP_MS);
fprintf('    tasa de muestreo .......... %.1f Hz  (req >= %d Hz)  %s\n', ...
        stL.k/tt(end), p.req.f_sensor_min, ok(stL.k/tt(end) >= p.req.f_sensor_min));
fprintf('    reloj del LLC ............. %.0f %% del tiempo real\n', ...
        100*(stL.clock_ms*1e-3)/tt(end));
fprintf('  HLC\n');
fprintf('    tasa de pose .............. %.1f Hz  (req >= %d Hz)  %s\n', ...
        rate, p.req.f_pose_min, ok(rate >= p.req.f_pose_min));
fprintf('    latencia p95 .............. %.1f ms  (req < %.0f ms)  %s\n', ...
        1000*pct(lat,95), 1000*p.req.latency_max, ok(pct(lat,95) < p.req.latency_max));
fprintf('  ENTREGA\n');
fprintf('    emitidas / usadas ......... %d / %d  (%.2f %%)  %s\n', ...
        stC.n_sent, stH.n_updates, 100*deliv, ok(deliv >= p.req.delivery_min));
fprintf('    perdidas en el enlace ..... %d\n', stC.n_lost);
fprintf('    rechazos del parser ....... %d\n', stH.n_badparse);
fprintf('    rechazos por plausibilidad. %d\n', stH.n_implaus);
fprintf('    sobrescrituras de buzon ... %d  (cuestan resolucion, no distancia)\n', ...
        stH.n_overwrite);
if useSL
    fprintf('  max |Simulink - MATLAB| ..... %.2e   (deberia ser ~0)\n', ...
            max(abs(XM(:)-XS(:))));
end
fprintf('=========================================================\n\n');

RES(run_i) = struct('dir', sentidos{run_i}, 'err_pct', err_pct, ...
    'dist', dist, 'her', rad2deg(herr(end)), 'lat', pct(lat,95), 'rate', rate);

%% ============ 6. GRAFICAS ============
figure('Color','w','Position',[60 60 1200 760]);

subplot(2,3,[1 4]); hold on; grid on; axis equal
plot(TRU(:,1), TRU(:,2), 'k-', 'LineWidth',2, 'DisplayName','verdad');
plot(XM(idx,1), XM(idx,2), 'r--','LineWidth',1.3,'DisplayName','EKF (HLC)');
title('Trayectoria'); xlabel('x [m]'); ylabel('y [m]'); legend('Location','best');

subplot(2,3,2); plot(tt(idx), 100*perr,'b'); grid on
title('Error de posicion'); xlabel('t [s]'); ylabel('[cm]');

subplot(2,3,3); hold on; grid on
plot(tt(idx), 1000*lat, 'm');
yline(1000*p.req.latency_max,'k:','requisito');
title('Latencia extremo a extremo'); xlabel('t [s]'); ylabel('[ms]');

subplot(2,3,5); hold on; grid on
plot(tt(idx), XM(idx,5),'b','DisplayName','b_\omega estimado');
yline(p.imu.gyro_bias_z,'k:','DisplayName','verdadero');
title('Sesgo del giroscopio (ZARU en reposo)');
xlabel('t [s]'); ylabel('[rad/s]'); legend('Location','best');

subplot(2,3,6); hold on; grid on
plot(tt, 1000*(stL.clock_ms*0 + tt), 'k:', 'DisplayName','tiempo real');
title('Reloj del LLC contra tiempo real');
xlabel('t real [s]'); ylabel('[s]'); legend('Location','best');

sgtitle(sprintf('Rover Olympus completo | LLC %s/%s | Tb = %.1f ms | %s', ...
        p.llc.clock_mode, p.llc.tx_mode, 1000*Tb, sentidos{run_i}));

end % for run_i

%% ============ 7. RESUMEN UMBmark ============
fprintf('\n================ RESUMEN UMBmark ================\n');
for i = 1:numel(RES)
    fprintf('  %-14s error %6.2f %% de %.2f m ; rumbo %7.2f deg\n', ...
            RES(i).dir, RES(i).err_pct, RES(i).dist, RES(i).her);
end
fprintf(['  El contraste entre sentidos separa Ed (diametros desiguales,\n' ...
         '  cambia de signo) de Eb (ancho de via, no cambia). Ninguno de\n' ...
         '  los dos mide la ESCALA de distancia: para eso, el ensayo de\n' ...
         '  recta.\n']);
fprintf('=================================================\n');


%% ===================== FUNCIONES LOCALES =====================
function s = ok(tf)
    if tf, s = 'CUMPLE'; else, s = 'NO CUMPLE'; end
end

function m = inwin(tt, win)
    m = zeros(numel(tt),1);
    for i = 1:size(win,1)
        m(tt >= win(i,1) & tt < win(i,2)) = 1;
    end
end

function v = pct(x, q)
    x = sort(x(~isnan(x)));
    if isempty(x), v = NaN; return; end
    v = x(max(1,min(numel(x), ceil(q/100*numel(x)))));
end

function a = wrapPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end
