function odom = sep_odom_empty()
%SEP_ODOM_EMPTY  Mensaje fusion -> estimacion, con todos sus campos.
%
%   odom = SEP_ODOM_EMPTY()
%
%   Es el espejo en MATLAB de pe_odom_msg_t (DESIGN.md, seccion 8.3). Todas
%   las magnitudes son ACUMULADAS: el agente de estimacion calcula las
%   diferencias contra lo ultimo que el mismo proceso, asi que un mensaje
%   perdido en el buzon solo cuesta resolucion temporal, no distancia.
%
%   Codegen-safe: define el tipo del struct de una sola vez.
%#codegen

    odom.n_s          = 0;      % tramas aceptadas por fusion (acumulado)
    odom.t_cum        = 0;      % [ms]    tiempo continuo del LLC
    odom.tick         = 0;      % [ms]    tick crudo de la trama mas reciente
    odom.S            = 0;      % [m]     distancia acumulada
    odom.TH           = 0;      % [rad]   giro acumulado de la odometria
    odom.OM           = 0;      % [rad/s] suma de tasas del giroscopio
    odom.n_g          = 0;      % muestras en OM
    odom.still_frames = 0;      % tramas consecutivas en reposo hasta esta
    odom.still        = false;  % still_frames >= n_still
    odom.rollover     = false;  % inclinacion por encima del umbral
    odom.llc_reset    = false;  % esta trama re-referencio por reinicio del LLC
    odom.reref        = false;  % esta trama re-referencio tras rechazos
    odom.imu_ok       = true;   % la IMU de esta trama es valida
    odom.w_gyro       = 0;      % [rad/s] tasa del giroscopio de esta trama
    odom.tilt         = 0;      % [rad]   inclinacion por acelerometro
end
