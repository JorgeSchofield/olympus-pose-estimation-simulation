function llc = llc_params()
%LLC_PARAMS  Modelo temporal y de sensores del LLC real (ATmega2560).
%
%   Reproduce el comportamiento del firmware v2.20 tal como esta escrito,
%   NO como esta documentado. Ref.: rover-low-level-controller @ 5a28ff2.
%
%   =====================================================================
%   EL HALLAZGO QUE GOBIERNA ESTE ARCHIVO
%   =====================================================================
%   El LLC no tiene reloj. main.rs cierra el ciclo con
%
%       elapsed_ms = elapsed_ms.wrapping_add(LOOP_MS);
%       arduino_hal::delay_ms(LOOP_MS);
%
%   No hay timer, ni millis(), ni captura de hardware: buscar TCNT, OCR0A o
%   micros() en todo el firmware no devuelve nada. Por tanto:
%
%     1) delay_ms al FINAL del ciclo es un HUECO, no un periodo. El ciclo
%        dura 20 ms MAS todo el trabajo. Los 50 Hz del indicador son
%        imposibles por construccion, se optimice lo que se optimice.
%     2) El timestamp declara 20 ms siempre. El reloj del LLC corre mas
%        lento que el real, y no con un factor constante: esta modulado
%        por el calendario de subtareas (periodos 5, 25 y 50 ciclos).
%
%   Por eso el emulador lleva DOS tiempos. t_real gobierna la planta;
%   t_llc es lo que el firmware cree y lo unico que ve el HLC.
%
%   =====================================================================
%   MODOS: LINEA BASE Y OBJETIVO
%   =====================================================================
%   clock_mode y tx_mode existen para poder cuantificar cuanto cuesta cada
%   defecto, en vez de afirmarlo. El contraste entre 'software'+'blocking'
%   (el LLC de hoy) y 'timer'+'interrupt' (el LLC propuesto) es evidencia
%   medible para justificar el cambio de firmware.
%
%     clock_mode 'software' : t_llc += LOOP_MS y el ciclo dura trabajo+delay
%     clock_mode 'timer'    : espera hasta fecha limite; periodo = LOOP_MS
%                             exacto y t_llc = t_real
%     tx_mode 'blocking'    : cada byte cuesta 10/baud (nb::block! en
%                             command_interface::send_response)
%     tx_mode 'interrupt'   : la transmision no bloquea el lazo
%
%   Los costes de subtarea son ESTIMACIONES. Medirlos es una prueba corta:
%   conmutar un GPIO al inicio del ciclo y mirarlo con osciloscopio. Hay
%   ademas una medida indirecta que quiza ya este en los logs del HLC:
%   engine.py sella cada TLM con time.monotonic(), asi que comparar el
%   delta de tick_ms contra el delta de time.monotonic entre TLM
%   consecutivos da el factor de escala del reloj sin instrumentar nada.

    % ---- lazo principal (config.rs) ----------------------------------
    llc.LOOP_MS     = 20;          % config.rs: LOOP_MS
    llc.HC_PERIOD   = 5;           % config.rs: HC_READ_PERIOD
    llc.SEN_PERIOD  = 25;          % config.rs: SEN_READ_PERIOD
    llc.TLM_PERIOD  = 50;          % config.rs: TLM_PERIOD
    llc.STALL_THRESH= 50;          % config.rs: STALL_THRESHOLD

    % ---- modos --------------------------------------------------------
    % Igual que baud: primero locales, el struct se rellena desde ellas.
    clock_mode     = 'software';   % 'software' (real hoy) | 'timer'
    tx_mode        = 'blocking';   % 'blocking'  (real hoy) | 'interrupt'
    llc.clock_mode = clock_mode;
    llc.tx_mode    = tx_mode;
    llc.emit_raw   = true;         % trama RAW cada ciclo (ruta a MATLAB)
    llc.emit_tlm   = true;         % TLM cada TLM_PERIOD ciclos

    % ---- enlace -------------------------------------------------------
    % OJO CODEGEN: baud vive primero en una variable LOCAL y el struct se
    % rellena desde ella. Escribir llc.byte_s = 10/llc.baud LEE el struct y
    % luego le ANADE un campo, y MATLAB Coder lo rechaza: "Code generation
    % does not support the addition of new fields after a structure has
    % been read or used". Un solo campo que infrinja la regla invalida
    % todos los que vengan detras, y el error aparece en el bloque de
    % Simulink que llamo a sim_params, muy lejos de la causa.
    baud           = 115200;       % USART0 -> USB. NO son 500 000.
    llc.baud       = baud;
    llc.byte_s     = 10/baud;      % 8N1: 10 bits por byte
    llc.raw_bytes  = 80;           % trama RAW tipica, ASCII
    llc.tlm_bytes  = 185;          % TLM extendida, ASCII

    % ---- costes de subtarea [s] (ESTIMADOS - medir con GPIO) ----------
    llc.c_imu      = 2.0e-3;       % una rafaga de 14 B por I2C software
                                   % (soft_i2c: semiperiodo 5 us, ~100 kHz)
    llc.c_hc       = 1.75e-3;      % config.rs: HC_ECHO_TIMEOUT_US
    llc.c_adc      = 13*8*104e-6;  % 13 canales x SEN_SAMPLES x ~104 us
    llc.c_misc     = 0.5e-3;       % MSM, rampa, logica de estado

    % ---- desfases DENTRO del ciclo [s] --------------------------------
    % La IMU y los encoders NO se leen en el mismo instante. En el firmware
    % actual la IMU se lee en el paso 1.5 y los contadores en el bloque de
    % stall, separados por todo el bloque de proximidad. Ese desfase viaja
    % en la misma trama como si fuera simultaneo.
    llc.off_imu    = 1.0e-3;       % desde el inicio del ciclo
    llc.off_enc    = 6.0e-3;

    % ---- reloj --------------------------------------------------------
    % Aunque se instale un timer, el cristal tiene error. El ATmega2560 de
    % un Arduino Mega usa resonador ceramico o cristal segun la variante;
    % el primero llega a +-0.5 %. Medible con el mismo ensayo de arriba.
    llc.clock_ppm  = 0;            % TBD - error de base de tiempo [ppm]

    % ---- IMU: MPU-9250 ------------------------------------------------
    % Mismo mapa de registros que el MPU-6050 para la rafaga 0x3B-0x48 y
    % las mismas escalas, de modo que el formato no cambia. WHO_AM_I = 0x71.
    llc.gyro_lsb   = 131;          % LSB/(deg/s) con GYRO_CONFIG = 0 (+-250)
    llc.accel_lsb  = 16384;        % LSB/g con ACCEL_CONFIG = 0 (+-2 g)
    llc.i16_max    = 32767;

    % DLPF. El driver viejo ponia CONFIG = 0x03. A 50 Hz de muestreo,
    % Nyquist esta en 25 Hz, de modo que un ancho de banda de ~41 Hz ALIASA
    % justo en la banda donde vibran los motores. Bajar a ~20 Hz cuesta
    % ~10 ms de retardo de grupo, un 10 % del presupuesto de latencia.
    % Es una decision abierta que este modelo puede responder.
    % VERIFICAR la tabla contra el register map antes de citarla.
    llc.dlpf_cfg   = 3;            % TBD
    llc.dlpf_bw    = 41;           % [Hz]  segun dlpf_cfg
    llc.dlpf_delay = 5.9e-3;       % [s]   retardo de grupo segun dlpf_cfg

    % ---- encoders -----------------------------------------------------
    % QuadratureEncoder::on_edge, disparada en CUALQUIER flanco del canal A
    % (EICRA = 0x55, EICRB = 0x05) con B para el signo: decodificacion x2.
    llc.enc_decode = 2;
    % Orden normativo de main.rs:83 y 849: FR FL CR CL RR RL.
    llc.idx_R      = [1 3 5];
    llc.idx_L      = [2 4 6];
    % El LLC suma las tres ruedas de cada lado antes de transmitir
    % (main.rs:895-896). Con side_sum = false el emulador transmite las seis
    % cuentas, para poder cuantificar que se perdio con esa decision.
    llc.side_sum   = true;

    % ---- banderas numericas equivalentes ------------------------------
    % Los bloques MATLAB Function del modelo completo las usan en vez de
    % strcmpi: comparar cadenas dentro de un bloque obliga a literales
    % entrecomillados, que se rompen al inyectar el codigo por script.
    % Derivadas, nunca editar a mano: editar clock_mode y tx_mode.
    llc.sw_clock = double(strcmpi(clock_mode, 'software'));
    llc.tx_block = double(strcmpi(tx_mode,    'blocking'));
end
