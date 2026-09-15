function p = rover_params()
%ROVER_PARAMS  Parametros del modelo de referencia del SEP (rover Olympus).
%
%   p = ROVER_PARAMS() devuelve la estructura maestra de parametros del
%   Subsistema de Estimacion de Pose, con procedencia y verificacion.
%
%   ARQUITECTURA DE PARAMETROS
%   --------------------------
%   Este archivo NO duplica valores. Envuelve a las dos funciones que si
%   son la fuente numerica y que ademas son codegen-safe:
%
%       sep_geo_params()  -> geometria y conversion de ticks
%       sep_ekf_params()  -> sintonizacion del filtro
%
%   El codigo generado para el HLC llama a esas dos. Este archivo anade lo
%   que solo existe en simulacion (planta, sensores, escenario) mas la capa
%   de estado y verificacion, que usa fprintf/warning y por tanto no puede
%   vivir del lado embebido.
%
%   REGLA DE USO
%   ------------
%   Ningun bloque del modelo debe contener una constante numerica. Todo
%   valor se referencia como p.<grupo>.<campo>.
%
%   ESTADO DE LOS VALORES (p.status)
%   --------------------------------
%     'MED'  medido y verificado en laboratorio
%     'DER'  derivado de otros parametros (no editar a mano)
%     'PROV' provisional, de la campana TRL-4 o de hoja de datos
%     'TBD'  pendiente de caracterizacion - Anexo B de DRT-SEP-001
%
%   Los resultados obtenidos con cualquier parametro en estado 'TBD' o
%   'PROV' NO son concluyentes y no sustentan el indicador PE-RNF-004.

% -------------------------------------------------------------------------
p.meta.doc     = 'DRT-SEP-001 v0.1';
p.meta.version = '0.2';
p.meta.updated = '2026-08-23';
p.status       = struct();

% =========================================================================
% 1. GEOMETRIA Y FILTRO  (fuente: funciones codegen-safe)
% =========================================================================
p.geo = sep_geo_params();
p.ekf = sep_ekf_params();

p.status.R_wheel         = 'MED';    % campana 14/09/2026, calibrador
p.status.ticks_per_rev   = 'MED';    % campana 14/09/2026, una vuelta a 3 velocidades
p.status.enc_sign        = 'MED';    % derechas negativas al avanzar (montaje en espejo)
p.status.B_nom           = 'MED';    % campana 14/09/2026, cinta metrica
p.status.chi             = 'TBD';    % Anexo B.2.6
p.status.B_eff           = 'DER';
p.status.k_rho           = 'PROV';
p.status.s2_theta        = 'PROV';
p.status.s2_w_enc        = 'PROV';
p.status.s2_bg           = 'PROV';
p.status.r_gyro          = 'PROV';
p.status.r_zaru          = 'PROV';
p.status.slip_thresh     = 'TBD';    % identificar con ensayo de rueda elevada
p.status.slip_gain       = 'PROV';
p.status.s2_ds_floor     = 'DER';

% =========================================================================
% 2. TRACCION Y CURVA PWM -> VELOCIDAD  (solo planta de simulacion)
% =========================================================================
p.drive.pwm_deadband = 0.15;    % [-] sin movimiento por debajo
p.drive.pwm_lin_lo   = 0.25;    % [-] inicio del tramo lineal
p.drive.pwm_lin_hi   = 0.75;    % [-] inicio de saturacion
p.drive.ticks_max    = 11257;   % [ticks/s] por rueda, en saturacion
p.drive.v_max        = 0.029;   % [m/s] velocidad en suelo al 100 % PWM
p.status.pwm_deadband= 'PROV';
p.status.pwm_lin_lo  = 'PROV';
p.status.pwm_lin_hi  = 'PROV';
p.status.ticks_max   = 'PROV';  % de la campana TRL-4, sin reconfirmar
p.status.v_max       = 'PROV';  % de la campana TRL-4; falta medir en suelo (M4)

% Hipotesis de trabajo sobre la velocidad, mientras el Anexo B no cierre.
%   'A' = usar v_max tal cual (0.029 m/s)
%   'B' = usar v_max x10, por si la medida en suelo tuviera error de unidades
p.drive.hypothesis   = 'A';
p.status.hypothesis  = 'TBD';

% =========================================================================
% 3. IMU - MPU-9250  (solo modelo de sensor en simulacion)
% =========================================================================
p.imu.gyro_lsb    = 131;      % [LSB/(deg/s)] fondo de escala +-250 deg/s
p.imu.accel_lsb   = 16384;    % [LSB/g] fondo de escala +-2 g
p.imu.gyro_bias_z = 0.012;    % [rad/s] sesgo tipico a modelar
p.imu.gyro_arw    = 0.01;     % [deg/s/sqrt(Hz)] densidad de ruido
p.imu.gyro_sf_err = 0.00;     % [-] error de factor de escala a inyectar
p.status.gyro_lsb    = 'PROV';
p.status.accel_lsb   = 'PROV';
p.status.gyro_bias_z = 'TBD';   % medir con rover en reposo, 60 s
p.status.gyro_arw    = 'PROV';  % hoja de datos; confirmar con varianza de Allan
p.status.gyro_sf_err = 'TBD';

% =========================================================================
% 4. TEMPORIZACION
% =========================================================================
p.time.T_llc      = 0.020;              % [s] ciclo del LLC (config.rs LOOP_MS)
p.time.f_sensor   = 50;                 % [Hz] Canal 2 (PE-RF-002)
p.time.f_pose     = 10;                 % [Hz] salida minima (PE-RNF-001)
p.time.lat_budget = 0.100;              % [s] PE-RNF-002
p.time.T_ekf      = 1/p.time.f_sensor;  % [s] paso del filtro
p.status.T_llc    = 'MED';
p.status.f_sensor = 'MED';
p.status.f_pose   = 'MED';
p.status.T_ekf    = 'DER';

% =========================================================================
% 5. ESCENARIO DE SIMULACION
% =========================================================================
p.sim.scenario   = 'umbmark';   % 'recta' | 'giro' | 'umbmark'
p.sim.side       = 2.0;         % [m] lado del cuadrado UMBmark
p.sim.w_turn     = 0.35;        % [rad/s] velocidad de giro en el sitio
p.sim.pause_s    = 2.0;         % [s] pausa inicial y final (habilita ZARU)
p.sim.slip_prob  = 0.01;        % [-] probabilidad de evento de patinaje por paso
p.sim.slip_frac  = 0.50;        % [-] fraccion de avance perdida al patinar
p.sim.disp_R     = 0.005;       % [-] dispersion relativa de radios (medida: <1 %)
p.sim.chi_true   = 1.20;        % [-] chi VERDADERO de la planta
p.sim.seed       = 20260823;

% GPS: se genera y se registra, pero NO entra al filtro (PE-CON-001).
p.sim.gps_on     = true;
p.sim.gps_cep    = 2.50;        % [m] error tipico del GY-GPSV3-NEO sin RTK
p.sim.gps_rate   = 1;           % [Hz]

% =========================================================================
% 6. VERIFICACION DE CONSISTENCIA
% =========================================================================
p.check = local_check(p);

end % rover_params


% -------------------------------------------------------------------------
function c = local_check(p)
%LOCAL_CHECK  Coherencia interna e inventario de parametros pendientes.

c.ok = true;
fprintf('\n=== rover_params: verificacion de consistencia ===\n');

% --- 1. Coherencia cinematica -------------------------------------------
% ticks_max, ticks_per_rev, R_wheel y v_max no son independientes. Tras la
% campana de caracterizacion, ticks_per_rev es un VECTOR (una cifra por
% rueda), de modo que el contraste se hace sobre el promedio.
R_mean   = mean(p.geo.R_wheel);
N_mean   = mean(p.geo.ticks_per_rev);
rev_s    = p.drive.ticks_max / N_mean;
v_implic = rev_s * 2*pi*R_mean;            % velocidad en vacio, rueda elevada
c.v_implicada = v_implic;
c.ratio       = v_implic / p.drive.v_max;

fprintf('  cuentas/vuelta (media) ........ %.0f (%s)\n', N_mean, p.status.ticks_per_rev);
fprintf('  dispersion entre ruedas ....... %.1f %%\n', ...
        (max(p.geo.ticks_per_rev)-min(p.geo.ticks_per_rev))/N_mean*100);
fprintf('  v implicada en vacio .......... %.4f m/s\n', v_implic);
fprintf('  v medida en suelo ............. %.4f m/s\n', p.drive.v_max);
fprintf('  razon vacio/suelo ............. x%.1f\n', c.ratio);

% La velocidad en vacio es NECESARIAMENTE mayor que la de suelo: bajo carga
% el motor cae de velocidad y hay deslizamiento. Una razon de 1 a 4 es
% esperable; fuera de ese rango hay algo mal.
if c.ratio < 1.0
    c.ok = false;
    warning('rover_params:velocidadIncoherente', ...
       ['La velocidad medida en suelo (%.3f m/s) supera a la implicada por ' ...
        'los encoders en vacio (%.3f m/s), lo que es fisicamente imposible. ' ...
        'Revisar ticks_max o v_max.'], p.drive.v_max, v_implic);
elseif c.ratio > 4.0
    c.ok = false;
    warning('rover_params:perdidaExcesiva', ...
       ['La velocidad en suelo es %.1f veces menor que la de vacio. Una ' ...
        'perdida tan grande sugiere deslizamiento severo o que ticks_max ' ...
        'no corresponde a la misma condicion de ensayo. Ejecutar M4.'], c.ratio);
end

% Dispersion entre ruedas: es el error sistematico Ed y no tiene
% explicacion geometrica si los diametros son iguales.
disp_N = (max(p.geo.ticks_per_rev)-min(p.geo.ticks_per_rev))/N_mean;
if disp_N > 0.05
    fprintf(['  AVISO: %.1f %% de dispersion en cuentas/vuelta entre ruedas.\n' ...
             '         Las cuentas por vuelta dependen del encoder y de la\n' ...
             '         reduccion, NO del diametro de la rueda, asi que esta\n' ...
             '         dispersion no tiene explicacion geometrica. Causas\n' ...
             '         posibles: error al juzgar "una vuelta" (repetir con\n' ...
             '         10 vueltas), reducciones distintas entre motores, o\n' ...
             '         deslizamiento de la llanta sobre el cubo.\n'], disp_N*100);
end

% --- 2. Coherencia del piso de cuantizacion ------------------------------
% s2_ds_floor vive en sep_ekf_params por la frontera de codegen, pero se
% deriva de la geometria. Si se recalibra ticks_per_rev hay que actualizarlo.
mpt_esperado = mean(abs(p.geo.m_per_tick));   % abs: lleva signo por lado
floor_esper  = mpt_esperado^2 / 12;
c.floor_ratio = p.ekf.s2_ds_floor / floor_esper;
if c.floor_ratio > 1.2 || c.floor_ratio < 0.8
    c.ok = false;
    warning('rover_params:pisoCuantizacionDesfasado', ...
       ['prm.s2_ds_floor (%.3e) no corresponde a la geometria actual ' ...
        '(%.3e). Actualizar sep_ekf_params.m tras recalibrar ticks_per_rev.'], ...
        p.ekf.s2_ds_floor, floor_esper);
end

% --- 3. Cuantizacion por muestra -----------------------------------------
v_cruise   = 0.5 * p.drive.v_max;
ticks_samp = (v_cruise / (2*pi*R_mean)) * N_mean * p.time.T_llc;
c.ticks_per_sample = ticks_samp;
fprintf('  ticks por muestra a media velocidad ... %.2f\n', ticks_samp);
if ticks_samp < 5
    fprintf(['  NOTA: menos de 5 ticks por muestra. No es critico mientras\n' ...
             '        el Canal 2 transporte cuentas ACUMULADAS y el HLC las\n' ...
             '        diferencie: asi el residuo sub-tick se arrastra y el\n' ...
             '        error de cuantizacion no se acumula. Si alguna vez se\n' ...
             '        transportaran incrementos ya cuantizados, aparece un\n' ...
             '        sesgo determinista de redondeo.\n']);
end

% --- 4. Chequeo de sanidad del filtro ------------------------------------
if p.ekf.P0(5) > 1e-4
    fprintf(['  AVISO: P0 del sesgo del giroscopio es grande. Un sesgo con\n' ...
             '         mucha libertad inicial absorbe el error de calibracion\n' ...
             '         del ancho de via durante el primer giro y anula el\n' ...
             '         aporte del giroscopio.\n']);
end

% --- 5. Inventario de pendientes -----------------------------------------
f = fieldnames(p.status);
v = struct2cell(p.status);
c.tbd  = f(strcmp(v, 'TBD'));
c.prov = f(strcmp(v, 'PROV'));
fprintf('  parametros TBD ................ %d\n', numel(c.tbd));
fprintf('  parametros provisionales ...... %d\n', numel(c.prov));
if ~isempty(c.tbd)
    c.ok = false;
    fprintf('  pendientes: %s\n', strjoin(c.tbd', ', '));
    fprintf(['  >> Resultados NO concluyentes para PE-RNF-004 mientras\n' ...
             '     existan parametros en estado TBD.\n']);
end
fprintf('==================================================\n\n');

end
