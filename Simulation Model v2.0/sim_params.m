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

    lp = llc_params();   % fuente unica del modelo de enlace y de sensores
    s.plant.dt_fine    = 0.001;                  % [s] rejilla del modo MATLAB
    s.plant.seg_hold   = 2.0;                    % [s] cola tras el ultimo segmento
    s.plant.half_track = [0.2905 0.2855 0.3570]; % [m] semivias F, C, R
    % chi VERDADERO de la planta. Se fija ligeramente por encima del valor
    % calibrado del filtro (geo.chi) a proposito: ese 1.7 % de diferencia
    % ES el residuo sistematico que UMBmark esta disenado para medir.
    % Igualarlos daria un cuadrado perfecto y la prueba no mediria nada;
    % separarlos mucho (chi_true = 1.20 contra chi = 1.00, como estaba)
    % saca al metodo de su rango de validez: el rover giraria 75 grados en
    % vez de 90 y el recorrido no volveria cerca del origen.
    % 2026-10-27: el giro sobre el rover da chi ~ 1.54 (B_eff ~ 0.96 m); la
    % planta usa ese valor y el filtro 1.50 (geo.chi), una diferencia del
    % ~3 % del mismo tipo que la descrita arriba.
    s.plant.chi_true   = 1.54;                   % [-] chi VERDADERO  PROV

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
    % byte_s NO se escribe aqui: es una propiedad del enlace y su fuente
    % unica es llc_params (que conoce el baudrate). Copiarlo a mano
    % permitiria que baud y byte_s se desincronizaran en silencio.
    s.chan.byte_s    = lp.byte_s;   % [s] tiempo de un byte, 8N1
    s.chan.p_loss    = 0.005;   % [-] perdida de trama (indicador admite 1 %)
    s.chan.p_corrupt = 0.000;   % [-] byte alterado. SIN CRC en la trama ASCII
    s.chan.poll_s    = 0.005;   % [s] sondeo del hilo de adquisicion
    s.chan.jitter_s  = 0.002;   % [s] jitter de planificacion de Linux
    s.chan.max_inflight = 16;   % tramas simultaneas en vuelo (ring del canal)

    % --- agentes del HLC ------------------------------------------------
    % gyro_scale y age_est los LEE hlc_step. Estaban definidos en
    % rover_params, que ya no es la fuente numerica: el bloque de Simulink
    % llama a sim_params directamente y no los veia. Mismo fallo que
    % chan.byte_s.
    s.agents.gyro_scale   = (pi/180) / lp.gyro_lsb;  % LSB -> rad/s
    s.agents.age_est      = 0.010;   % [s] edad supuesta de la muestra al
                                     % llegar. El HLC NO puede conocerla: la
                                     % trama no lleva el instante real de
                                     % muestreo, solo el contador de software.
    % SEMANTICA DEL BUZON de CMAES. Profundidad 1: xQueueCreate(1, ...).
    %   'discard_new' : lo que CMAES hace HOY. Agent_Msg::send es
    %                   xQueueSend con timeout = 0, asi que ante buzon
    %                   lleno muere el mensaje NUEVO y el consumidor recibe
    %                   el mas VIEJO. Es el valor por omision porque es el
    %                   comportamiento real, no el deseable.
    %   'overwrite'   : lo que conviene para datos de sensor, donde la
    %                   muestra reciente vale mas que la vieja. NO EXISTE en
    %                   CMAES: habria que anadir MAES_QueueOverwrite a la
    %                   capa de portabilidad, junto con la rotacion de N
    %                   buffers (CMAES pasa punteros, no copias).
    %   'block'       : xQueueSend con timeout > 0. NO USAR en la ruta de
    %                   sensores: detiene al agente de adquisicion, que deja
    %                   de drenar el puerto serial, y las tramas se pierden
    %                   en el buffer del tty. Convierte presion de buzon en
    %                   fallo del indicador de ENTREGA.
    %
    % Con el agente de estimacion mas rapido que la llegada de tramas el
    % buzon casi no se llena y las tres son indistinguibles. La diferencia
    % aparece cuando se igualan las tasas, que es justo lo que provoca
    % arreglar el reloj del LLC: las dos correcciones interactuan.
    % La cadena es para leerla; el codigo usa la bandera NUMERICA. Igual
    % que sw_clock y tx_block en llc_params: comparar cadenas dentro de un
    % bloque MATLAB Function obliga a llevar campos char en el struct de
    % parametros, que codegen trata como arreglos de tamano variable.
    % OJO: mbox_policy vive en una LOCAL primero. Escribir
    % s.agents.mbox_mode = f(s.agents.mbox_policy) leeria el struct y luego
    % le anadiria un campo, que codegen rechaza.
    mbox_policy = 'discard_new';        % 'discard_new' | 'overwrite' | 'block'
    if     strcmpi(mbox_policy, 'overwrite'), mbox_mode = 1;
    elseif strcmpi(mbox_policy, 'block'),     mbox_mode = 2;
    else,                                     mbox_mode = 0;
    end
    s.agents.mbox_policy  = mbox_policy;
    s.agents.mbox_mode    = mbox_mode;   % 0 descarta nuevo, 1 sobrescribe, 2 bloquea
    % ACTIVACION DEL AGENTE DE ESTIMACION.
    %   'periodic' : despierta cada est_period (como en el modelo 2.0 y en
    %                el cuadro 3.7 de la tesis).
    %   'event'    : despierta al llegar un mensaje al buzon 2 (DESIGN.md,
    %                decision D3). Es lo que HACE la aplicacion CMAES, y por
    %                eso es el valor por omision desde 2026-10-21: el modelo
    %                y la aplicacion estiman de la misma manera. 'periodic'
    %                queda solo para reproducir el cuadro 3.7 de la tesis.
    % Misma regla que mbox_policy: la cadena en una LOCAL, la bandera
    % numerica es lo que lee hlc_step.
    est_policy = 'event';               % 'event' (aplicacion) | 'periodic'
    if strcmpi(est_policy, 'event'), est_mode = 1; else, est_mode = 0; end
    s.agents.est_policy = est_policy;
    s.agents.est_mode   = est_mode;
    s.agents.est_period = 0.020;  % [s] periodo del agente de estimacion
                                  % (solo est_mode = 0). Si excede el
                                  % intervalo entre mensajes, el buzon 2
                                  % descarta.
    s.agents.est_delay  = 0.003;  % [s] coste del ciclo del EKF
    s.agents.com_delay  = 0.002;  % [s] empaquetado y publicacion
    s.agents.jitter     = 0.002;  % [s] jitter por agente
    % --- agente de fusion: plausibilidad y reposo -----------------------
    % dt_reject subio de 0.5 s a 2.0 s: con mensajes acumulados un hueco de
    % hasta 2 s trae su distancia EXACTA, y sep_estimation_step lo integra
    % en subpasos. Por encima se considera tick corrupto.
    s.agents.dt_reject     = 2.000;  % [s] salto de tick implausible
    s.agents.dcount_reject = 20000;  % [cuentas por loop_ms] salto implausible
    s.agents.loop_ms       = lp.LOOP_MS;  % [ms] periodo nominal del LLC
    s.agents.reset_window  = 1000;   % [cuentas] |acumulador| tras un reinicio
    s.agents.k_reref       = 3;      % rechazos seguidos -> re-referenciar
    s.agents.n_still       = 1;      % tramas quietas para declarar reposo
                                     % (1 = como el modelo 2.0; subir si el
                                     % rebote del Hall da cuentas en reposo)
    s.agents.tilt_rollover = deg2rad(30);  % [rad] umbral de vuelco
    % --- entradas de la fusion (hlc_fusion_inputs = pe_fusion_inputs) -----
    % Se exportan a pose.conf como est.dt_source y est.gyro_yaw_axis, de
    % modo que el modelo y la aplicacion usan los mismos valores.
    s.agents.dt_source     = 0;      % base de tiempo: 0 tick del LLC (tesis
                                     % 3.2.4), 1 llegada al HLC. Con el
                                     % firmware actual (reloj por software,
                                     % k ~ 1.87) las pruebas en vivo usan 1
    s.agents.gyro_yaw_axis = 2;      % eje de la IMU hacia arriba: 0 x, 1 y, 2 z
    % --- agente de estimacion ---------------------------------------------
    s.agents.t_gap_max     = 10.0;   % [s] hueco mayor: re-referenciar
    % --- solo pruebas ------------------------------------------------------
    s.agents.test_drop_every = 0;    % >0: descarta 1 de cada N mensajes del
                                     % buzon 2 (test_sep_model, caso 11)

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
