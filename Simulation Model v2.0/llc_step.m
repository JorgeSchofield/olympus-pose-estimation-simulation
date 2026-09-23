function [st, buf, n, emit] = llc_step(st, w_true, a_body, pitch, roll, arc, ...
                                       noise, llc, geo, imu, Tb)
%LLC_STEP  Un paso base del lazo principal del LLC. Fuente unica.
%
%   [st, buf, n, emit] = LLC_STEP(...)
%
%   Se llama cada Tb segundos (paso base del modelo, tipicamente 0.5 ms) y
%   decide internamente que le toca hacer. Asi un ciclo de duracion
%   VARIABLE se representa como un numero variable de pasos base, y el
%   jitter que se quiere estudiar sobrevive intacto.
%
%   Cuantizacion: los ciclos quedan redondeados a multiplos de Tb. Con
%   Tb = 0.5 ms y ciclos de ~30 ms eso es un 1.6 %, muy por debajo del
%   ~50 % de error de reloj que es el objeto de estudio, pero significa que
%   el modelo NO puede resolver efectos temporales mas finos que Tb. El
%   tiempo de un byte a 115200 son 86.8 us, asi que la transmision se
%   modela a nivel de trama, no de byte.
%
%   =====================================================================
%   EL COMPORTAMIENTO QUE ESTE BLOQUE REPRODUCE
%   =====================================================================
%   main.rs cierra el ciclo con
%       elapsed_ms = elapsed_ms.wrapping_add(LOOP_MS);
%       arduino_hal::delay_ms(LOOP_MS);
%   No hay timer en todo el firmware. Por tanto delay_ms al FINAL es un
%   HUECO, no un periodo: el ciclo dura LOOP_MS MAS el trabajo, mientras el
%   timestamp declara LOOP_MS siempre.
%
%   st.clock_ms es lo que el firmware cree. El HLC no ve nada mas.
%
%   ESTADO st
%     .phase      pasos base transcurridos en el ciclo actual
%     .len        duracion del ciclo actual, en pasos base
%     .k          numero de ciclo (gobierna el calendario 5 / 25 / 50)
%     .clock_ms   reloj del firmware [ms]
%     .lat_gyr .lat_acc   ultima lectura de IMU latcheada
%     .lat_cnt   ultimas cuentas latcheadas (1x6)
%     .last_cnt .stall_t  deteccion de stall
%     .p_imu .p_enc       fase, en pasos base, de cada lectura
%     .t_real             tiempo real acumulado [s], solo diagnostico
%
%   SALIDA
%     buf, n : trama de bytes y su longitud. Validos solo si emit = true.
%     emit   : true en el paso base en que termina el ciclo.
%#codegen

    buf  = zeros(1, raw_buf_max(), 'uint8');
    n    = 0;
    emit = false;

    % --- inicio de ciclo: calcular su duracion -------------------------
    if st.phase == 0
        st.k = st.k + 1;

        work = llc.c_misc + llc.c_imu;
        if mod(st.k, llc.HC_PERIOD)  == 0, work = work + llc.c_hc;  end
        if mod(st.k, llc.SEN_PERIOD) == 0, work = work + llc.c_adc; end

        tx = 0;
        if llc.emit_raw,                                tx = tx + llc.raw_bytes; end
        if llc.emit_tlm && mod(st.k, llc.TLM_PERIOD)==0, tx = tx + llc.tlm_bytes; end
        if strcmpi(llc.tx_mode, 'blocking')
            work = work + tx*llc.byte_s;
        end

        if strcmpi(llc.clock_mode, 'software')
            % delay_ms es un hueco: periodo = trabajo + LOOP_MS
            cyc = work + llc.LOOP_MS*1e-3;
        else
            % espera a fecha limite: periodo = LOOP_MS, salvo sobrecarga
            cyc = max(work, llc.LOOP_MS*1e-3);
        end
        st.len = max(1, round(cyc/Tb));

        st.p_imu = min(max(1, round(llc.off_imu/Tb)), st.len);
        st.p_enc = min(max(1, round(llc.off_enc/Tb)), st.len);
    end

    st.phase = st.phase + 1;

    % --- lectura de la IMU ---------------------------------------------
    % Ocurre en un punto del ciclo; los encoders en otro. La trama los
    % transporta como si fueran simultaneos y el HLC no puede saberlo.
    if st.phase == st.p_imu
        [st.lat_gyr, st.lat_acc] = imu_read(w_true, a_body, pitch, roll, ...
                                            noise, llc, imu, st.t_real);
        st.t_imu = st.t_real;
    end

    % --- lectura de los encoders ----------------------------------------
    if st.phase == st.p_enc
        st.lat_cnt = arc_to_counts(arc, geo);
        st.t_enc   = st.t_real;

        % Deteccion de stall (main.rs paso 3)
        moved = (st.lat_cnt ~= st.last_cnt);
        for i = 1:6
            if moved(i), st.stall_t(i) = 0;
            else,        st.stall_t(i) = st.stall_t(i) + 1;
            end
        end
        st.last_cnt = st.lat_cnt;
    end

    % --- fin de ciclo: emitir -------------------------------------------
    if st.phase >= st.len
        % Suma por lado (main.rs:895-896). El LLC solo expone estas dos
        % cifras: las cuentas individuales no son legibles desde el HLC.
        encR = sum(st.lat_cnt(geo.idx_R));
        encL = sum(st.lat_cnt(geo.idx_L));

        [buf, n] = raw_frame_bytes(st.clock_ms, st.lat_acc, st.lat_gyr, encL, encR);
        emit = true;

        if strcmpi(llc.clock_mode, 'software')
            st.clock_ms = mod(st.clock_ms + llc.LOOP_MS, 2^32);
        else
            st.clock_ms = mod((st.t_real + st.len*Tb)*1000 / ...
                              (1 + llc.clock_ppm*1e-6), 2^32);
        end

        st.phase = 0;
    end

    st.t_real = st.t_real + Tb;
end

% =====================================================================
function cnt = arc_to_counts(arc, geo)
%ARC_TO_COUNTS  Arco de rueda -> cuentas ACUMULADAS del encoder.
%   fix() y no floor(): con m_per_tick negativo en el lado derecho (montaje
%   en espejo) floor() truncaria hacia -inf y sesgaria ese lado.
%   Se cuantiza el ACUMULADO, no el incremento: asi el residuo sub-tick se
%   arrastra de un ciclo al siguiente, que es como se comporta un encoder
%   fisico y lo que mantiene acotado el error de cuantizacion.
%#codegen
    mpt = zeros(1,6);
    for k = 1:3
        iR = geo.idx_R(k);  iL = geo.idx_L(k);
        mpt(iR) = geo.sign_R * 2*pi*geo.R_wheel(iR) / geo.ticks_per_rev(iR);
        mpt(iL) = geo.sign_L * 2*pi*geo.R_wheel(iL) / geo.ticks_per_rev(iL);
    end
    cnt = fix(arc ./ mpt);
    cnt = mod(cnt + 2^31, 2^32) - 2^31;     % los acumuladores son i32
end

% =====================================================================
function [gyr, acc] = imu_read(w_body, a_body, pitch, roll, noise, llc, imu, t)
%IMU_READ  Verdad -> lecturas crudas int16 del MPU-9250.
%
%   El acelerometro mide FUERZA ESPECIFICA, no aceleracion: en reposo y
%   nivelado el eje vertical lee +1 g. Por eso la gravedad se suma aqui,
%   proyectada segun la orientacion.
%
%   El giroscopio mide en el marco del CUERPO. Con cabeceo, su componente z
%   no es la tasa de guinada del mundo: falta un factor cos(pitch). A 15
%   grados eso es un 3.5 % de error sistematico de rumbo.
%
%   noise (1x5) llega de fuera y no se genera aqui. Es deliberado: con las
%   secuencias de ruido inyectadas como senal, el bloque de Simulink y el
%   bucle de MATLAB dan resultados IDENTICOS y max|Simulink - MATLAB|
%   vuelve a ser una metrica con sentido. Dos implementaciones que
%   coinciden son evidencia; una sola es solo codigo.
%#codegen
    g = 9.80665;

    wz   = w_body * cos(pitch);
    bias = imu.gyro_bias_z + imu.gyro_bias_drift * t;
    sg   = deg2rad(imu.gyro_arw) * sqrt(max(llc.dlpf_bw,1));
    wz_m = wz*(1 + imu.gyro_sf_err) + bias + sg*noise(1);

    gyr = [sat(round(noise(2)*llc.gyro_lsb*0.01), llc.i16_max), ...
           sat(round(noise(3)*llc.gyro_lsb*0.01), llc.i16_max), ...
           sat(round(rad2deg(wz_m)*llc.gyro_lsb), llc.i16_max)];

    ax = a_body(1) + g*sin(pitch);
    ay = a_body(2) - g*sin(roll)*cos(pitch);
    az =             g*cos(pitch)*cos(roll);

    sa = imu.accel_nd * sqrt(max(llc.dlpf_bw,1));
    acc = [sat(round((ax + sa*noise(4))/g * llc.accel_lsb), llc.i16_max), ...
           sat(round((ay + sa*noise(5))/g * llc.accel_lsb), llc.i16_max), ...
           sat(round( az               /g * llc.accel_lsb), llc.i16_max)];
end

function v = sat(v, lim)
%#codegen
    if v >  lim, v =  lim; end
    if v < -lim, v = -lim; end
end
