function s = sim_params()
%SIM_PARAMS  Numeros de la simulacion: planta, canal, agentes, escenario.
%
%   Fuente numerica de todo lo que existe SOLO en simulacion. No imprime ni
%   avisa, de modo que los bloques de Simulink pueden llamarla y el
%   generador la pliega como constante.
%
%   rover_params.m la envuelve anadiendo procedencia y verificacion; esa
%   capa usa fprintf/warning y por eso no puede vivir aqui. Ningun valor se
%   duplica entre los dos archivos.

    % --- planta ---------------------------------------------------------
    s.plant.dt_fine    = 0.001;                  % [s] rejilla del modo MATLAB
    s.plant.seg_hold   = 2.0;                    % [s] cola tras el ultimo segmento
    s.plant.half_track = [0.2905 0.2855 0.3570]; % [m] semivias F, C, R
    s.plant.chi_true   = 1.20;                   % [-] chi VERDADERO  TBD

    % Deslizamiento: tres mecanismos separados, cada uno ejercita una parte
    % distinta. Ventanas en [t_ini t_fin] segundos.
    s.plant.slip.common_frac = 0.00;  s.plant.slip.common_win = zeros(0,2);
    s.plant.slip.side_frac   = 0.00;  s.plant.slip.side_win   = zeros(0,2);
    s.plant.slip.wheel_frac  = 0.00;  s.plant.slip.wheel_win  = zeros(0,2);
    s.plant.slip.wheel_idx   = 1;                % FR

    % Terreno. Importa por dos razones distintas: la gravedad proyectada
    % domina al acelerometro (0.17 m/s^2 por grado, contra 0.15 m/s^2 de
    % aceleracion propia TOTAL de este rover), y el giroscopio mide en el
    % marco del cuerpo, de modo que con cabeceo falta 1/cos(pitch).
    s.plant.terrain.pitch_dc  = 0.0;
    s.plant.terrain.pitch_amp = deg2rad(1.5);
    s.plant.terrain.pitch_f   = 0.4;             % [Hz] cabeceo rocker-bogie
    s.plant.terrain.roll_dc   = 0.0;
    s.plant.terrain.roll_amp  = deg2rad(1.0);
    s.plant.terrain.roll_f    = 0.27;

    % --- traccion -------------------------------------------------------
    s.drive.v_max  = 0.029;     % [m/s] al 100 % PWM, medido en suelo
    s.drive.w_turn = 0.35;      % [rad/s] giro en el sitio

    % --- IMU MPU-9250 ---------------------------------------------------
    s.imu.gyro_bias_z     = 0.012;    % [rad/s]               TBD
    s.imu.gyro_bias_drift = 0.0;      % [rad/s^2] deriva      TBD
    s.imu.gyro_arw        = 0.01;     % [deg/s/sqrt(Hz)]
    s.imu.gyro_sf_err     = 0.00;     % [-]                   TBD
    s.imu.accel_nd        = 2.94e-3;  % [m/s^2/sqrt(Hz)]

    % --- canal serial (USART0 -> USB, 115200 8N1) -----------------------
    s.chan.p_loss    = 0.005;   % [-] perdida de trama (indicador admite 1 %)
    s.chan.p_corrupt = 0.000;   % [-] byte alterado. SIN CRC en la trama ASCII
    s.chan.poll_s    = 0.005;   % [s] sondeo del hilo de adquisicion
    s.chan.jitter_s  = 0.002;   % [s] jitter de planificacion de Linux
    s.chan.max_inflight = 16;   % tramas simultaneas en vuelo (ring del canal)

    % --- agentes del HLC ------------------------------------------------
    s.agents.est_period = 0.020;  % [s] periodo del agente de estimacion.
                                  % Si excede el intervalo entre tramas, el
                                  % buzon de profundidad 1 sobrescribe.
    s.agents.est_delay  = 0.003;  % [s] coste del ciclo del EKF
    s.agents.com_delay  = 0.002;  % [s] empaquetado y publicacion
    s.agents.jitter     = 0.002;  % [s] jitter por agente
    s.agents.dt_reject  = 0.500;  % [s] dt implausible -> descartar
    s.agents.dcount_reject = 20000;

    % --- escenario ------------------------------------------------------
    s.sim.scenario = 'umbmark';   % 'recta' | 'giro' | 'umbmark'
    s.sim.side     = 2.0;         % [m] lado del cuadrado
    s.sim.pause_s  = 3.0;         % [s] pausas. NO son cortesia: son lo unico
                                  % que hace observable el sesgo via ZARU.
    s.sim.seed     = 20260917;
    s.sim.Tb       = 0.0005;      % [s] paso base del modelo completo

    % GPS: se genera y se registra, pero NUNCA entra al filtro.
    s.sim.gps_on   = true;
    s.sim.gps_cep  = 2.50;        % [m] GY-GPSV3-NEO sin RTK
    s.sim.gps_rate = 1;           % [Hz]

    % --- trama ----------------------------------------------------------
    % main.rs declara raw_buf: [u8; 100] y envia &raw_buf[..r_i]. El modelo
    % usa el mismo tamano: buffer fijo de 100 bytes mas longitud util.
    s.frame.buf_len = 100;

    % --- indicadores del proyecto ---------------------------------------
    s.req.f_sensor_min = 50;      % [Hz]
    s.req.f_pose_min   = 10;      % [Hz]
    s.req.latency_max  = 0.100;   % [s]
    s.req.delivery_min = 0.99;    % [-]
    s.req.err_max_pct  = 3.0;     % [%] del recorrido
end
