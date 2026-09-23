%% RUN_SEP_DEMO  Modelo de SOLO ESTIMACION - entrada al modelo Simulink corto.
%
%  Rover Olympus - TFG Ingenieria Electronica ITCR / SETEC.
%
%  Genera la cadena planta -> LLC -> canal con las funciones de paso base,
%  extrae las senales que consume el agente de estimacion y las deja en el
%  workspace para build_sep_model. Sirve para trabajar el filtro sin cargar
%  con los 600 000 pasos del modelo completo.
%
%  Para la cadena entera con los dos controladores: run_sep_full_demo.
%
%  Ejecutar:  >> run_sep_demo

clear; clc; close all;

p  = rover_params();
rng(p.sim.seed);
Tb = p.sim.Tb;

%% ============ 1. PERFIL DE COMANDO ============
% Las pausas NO son cortesia: son lo unico que hace observable el sesgo del
% giroscopio via ZARU. Un recorrido sin paradas lo deja sin anclar.
v = p.drive.v_max;  w = p.drive.w_turn;  tp = p.sim.pause_s;
t_side = p.sim.side/v;  t_turn = (pi/2)/w;
seg = [0 0 0];  t = tp;
for i = 1:4
    seg(end+1,:) = [t v 0];   t = t + t_side; %#ok<SAGROW>
    seg(end+1,:) = [t 0 0];   t = t + tp;     %#ok<SAGROW>
    seg(end+1,:) = [t 0 w];   t = t + t_turn; %#ok<SAGROW>
    seg(end+1,:) = [t 0 0];   t = t + tp;     %#ok<SAGROW>
end
N  = round(t/Tb);
tt = (0:N-1).'*Tb;

%% ============ 2. CADENA PLANTA -> LLC -> CANAL ============
tr = p.plant.terrain;
pitch = tr.pitch_dc + tr.pitch_amp*sin(2*pi*tr.pitch_f*tt);
roll  = tr.roll_dc  + tr.roll_amp *sin(2*pi*tr.roll_f *tt + 1.1);
noise = randn(N,5);
uch   = rand(N,3);

stP = plant_init();  stL = llc_init();  stC = channel_init();
TRU = zeros(N,3);
ds_l = []; dth_l = []; gy_l = []; dt_l = []; mv_l = []; tm_l = [];
ref_tick = NaN; ref_L = NaN; ref_R = NaN;

for k = 1:N
    j = find(seg(:,1) <= tt(k), 1, 'last');
    [stP, o] = plant_step(stP, seg(j,2), seg(j,3), zeros(1,6), ...
                          pitch(k), roll(k), Tb, p.plant);
    TRU(k,:) = o.pose;
    [stL, b, nb, em] = llc_step(stL, o.w, o.a_body, pitch(k), roll(k), ...
                                o.arc, noise(k,:), p.llc, p.geo, p.imu, Tb);
    [stC, cb, cn, ar] = channel_step(stC, b, nb, em, uch(k,:), p.chan, Tb);

    if ar
        [f, ok] = raw_frame_from_bytes(cb, cn);
        if ok
            if isnan(ref_tick)
                ref_tick = f.tick_ms; ref_L = f.encL; ref_R = f.encR;
            else
                dt  = mod(f.tick_ms - ref_tick, 2^32)*1e-3;
                dCL = mod(f.encL - ref_L + 2^31, 2^32) - 2^31;
                dCR = mod(f.encR - ref_R + 2^31, 2^32) - 2^31;
                ref_tick = f.tick_ms; ref_L = f.encL; ref_R = f.encR;
                [dsv, dthv, movv] = sep_odometry(dCR, dCL, p.geo);
                tm_l(end+1,1)  = tt(k);                       %#ok<SAGROW>
                ds_l(end+1,1)  = dsv;                         %#ok<SAGROW>
                dth_l(end+1,1) = dthv;                        %#ok<SAGROW>
                gy_l(end+1,1)  = f.gyr(3)*p.agents.gyro_scale;%#ok<SAGROW>
                dt_l(end+1,1)  = dt;                          %#ok<SAGROW>
                mv_l(end+1,1)  = movv;                        %#ok<SAGROW>
            end
        end
    end
end

% Senales para los bloques From Workspace de build_sep_model.
ds_log   = [tm_l ds_l];    %#ok<NASGU>
dth_log  = [tm_l dth_l];   %#ok<NASGU>
gyro_log = [tm_l gy_l];    %#ok<NASGU>
dt_log   = [tm_l dt_l];    %#ok<NASGU>
mov_log  = [tm_l mv_l];    %#ok<NASGU>

%% ============ 3. EKF EN MATLAB (referencia) ============
prm = sep_ekf_params();
x = zeros(5,1);  P = diag(prm.P0);
M = numel(tm_l);
EST = zeros(M,5);  DIAG = zeros(M,3);
for k = 1:M
    [x, P, xe, dg] = sep_ekf_step(x, P, ds_l(k), dth_l(k), gy_l(k), ...
                                  dt_l(k), mv_l(k), prm);
    EST(k,:) = xe.';  DIAG(k,:) = dg.';
end

%% ============ 4. SIMULINK ============
modelName = 'sep_ekf_rover';
useSL = true;  EST_sl = [];
try
    build_sep_model(modelName, p.llc.LOOP_MS*1e-3);
    set_param(modelName, 'StopTime', num2str(tm_l(end)));
    so = sim(modelName);
    EST_sl = squeeze(so.get('x_est_log').Data).';
    if size(EST_sl,1) ~= M
        EST_sl = interp1(so.get('x_est_log').Time, EST_sl, tm_l);
    end
catch ME
    useSL = false;
    warning(['El modelo Simulink no se ejecuto (%s). Se usa el bucle de ' ...
             'MATLAB. Para ver la causa real, ejecutar sim(''%s'') sin ' ...
             'try/catch.'], ME.message, modelName);
end

%% ============ 5. METRICAS ============
TR   = interp1(tt, TRU, tm_l, 'linear', 'extrap');
perr = hypot(EST(:,1)-TR(:,1), EST(:,2)-TR(:,2));
herr = abs(wrapPiLocal(EST(:,3)-TR(:,3)));
dist = stP.dist;

fprintf('\n---------- SOLO ESTIMACION ----------\n');
fprintf('  LLC: reloj %s, TX %s\n', p.llc.clock_mode, p.llc.tx_mode);
fprintf('  ciclo real medio ........... %.2f ms (el firmware dice %.0f ms)\n', ...
        1000*tt(end)/max(stL.k,1), p.llc.LOOP_MS);
fprintf('  dt que el filtro creyo ..... %.2f ms\n', 1000*mean(dt_l));
fprintf('  distancia recorrida ........ %.2f m\n', dist);
fprintf('  error final ................ %.2f cm = %.2f %% (req <= %.0f %%)\n', ...
        100*perr(end), 100*perr(end)/dist, p.req.err_max_pct);
fprintf('  error de rumbo final ....... %.2f deg\n', rad2deg(herr(end)));
fprintf('  sesgo giro est/verdadero ... %.5f / %.5f rad/s\n', ...
        EST(end,5), p.imu.gyro_bias_z);
if useSL
    fprintf('  max |Simulink - MATLAB| .... %.2e  (deberia ser ~0)\n', ...
            max(abs(EST(:)-EST_sl(:))));
end

%% ============ 6. GRAFICAS ============
figure('Color','w','Position',[60 60 1150 700]);
subplot(2,2,[1 3]); hold on; grid on; axis equal
plot(TRU(:,1), TRU(:,2),'k-','LineWidth',2,'DisplayName','verdad');
plot(EST(:,1), EST(:,2),'r--','LineWidth',1.3,'DisplayName','EKF');
title('Trayectoria'); xlabel('x [m]'); ylabel('y [m]'); legend('Location','best');

subplot(2,2,2); plot(tm_l, 100*perr,'b'); grid on
title('Error de posicion'); xlabel('t [s]'); ylabel('[cm]');

subplot(2,2,4); hold on; grid on
plot(tm_l, EST(:,5),'b','DisplayName','b_\omega estimado');
yline(p.imu.gyro_bias_z,'k:','DisplayName','verdadero');
title('Sesgo del giroscopio (ZARU en reposo)');
xlabel('t [s]'); ylabel('[rad/s]'); legend('Location','best');

sgtitle(sprintf('SEP solo estimacion | LLC %s/%s', ...
        p.llc.clock_mode, p.llc.tx_mode));

function a = wrapPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end
