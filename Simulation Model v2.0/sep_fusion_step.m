function [fst, odom, emit] = sep_fusion_step(fst, tick, encL, encR, gz_lsb, acc, imu_ok, geo, ag)
%SEP_FUSION_STEP  Un ciclo del agente de fusion de datos.
%
%   [fst, odom, emit] = SEP_FUSION_STEP(fst, tick, encL, encR, gz_lsb, acc,
%                                       imu_ok, geo, ag)
%
%   Entradas (una trama RAW ya decodificada, SIN convertir):
%     fst     : estado, de sep_fusion_init()
%     tick    : [ms] contador del LLC (u32)
%     encL/R  : acumuladores por lado (i32, envuelven)
%     gz_lsb  : giroscopio de yaw crudo [LSB]
%     acc     : acelerometro crudo [LSB], 1x3, +1 g en z con el rover nivelado
%     imu_ok  : la lectura de la IMU de esta trama es valida
%     geo     : struct de sep_geo_params()
%     ag      : parametros de los agentes (sim_params().agents). Lee:
%               gyro_scale, dt_reject, dcount_reject, loop_ms,
%               reset_window, k_reref, n_still, tilt_rollover
%
%   Salidas:
%     fst  : estado actualizado
%     odom : mensaje para el agente de estimacion (sep_odom_empty)
%     emit : true si hay mensaje que enviar (false si la trama se rechazo)
%
%   POR QUE EL MENSAJE LLEVA MAGNITUDES ACUMULADAS
%   ----------------------------------------------
%   Es la regla de los acumuladores aplicada un eslabon mas adelante: entre
%   fusion y estimacion tambien hay un buzon de profundidad 1 que puede
%   descartar mensajes. Si el mensaje llevara incrementos (ds, dth), uno
%   descartado seria distancia perdida para siempre. Con S y TH acumulados,
%   el agente de estimacion diferencia contra lo ultimo que proceso y el
%   mensaje siguiente trae el avance completo. La conversion es lineal, asi
%   que S = sum(ds) es exactamente (lambda_R*C_R + lambda_L*C_L)/2 sobre los
%   acumuladores desenvueltos (DESIGN.md, decision D2).
%
%   PLAUSIBILIDAD
%   -------------
%   La trama ASCII no lleva CRC: un byte alterado da un numero creible. Se
%   rechazan saltos de tiempo o de cuentas imposibles, y la referencia sigue
%   siendo la ULTIMA TRAMA ACEPTADA. Asi el avance de la trama rechazada
%   llega integro con la siguiente buena. (hlc_step re-referenciaba con la
%   trama rechazada y perdia ~2 intervalos de distancia por rechazo.)
%   Tras ag.k_reref rechazos seguidos se asume una discontinuidad real y se
%   re-referencia sin integrar el salto.
%#codegen

    odom = sep_odom_empty();
    emit = false;

    w_m  = double(gz_lsb) * ag.gyro_scale;
    tilt = local_tilt(acc);

    % --- primera trama: solo referencia ---------------------------------
    if ~fst.have_ref
        fst = local_reference(fst, tick, encL, encR);
        fst.have_ref = true;
        fst.still_frames = 0;
        odom = local_pack(fst, tick, w_m, tilt, imu_ok, ag);
        emit = true;
        return;
    end

    % --- reinicio del LLC: el tick retrocede y los acumuladores vuelven a ~0
    %     No se integra el salto. S, TH y OM siguen continuos; el tiempo
    %     avanza un periodo nominal porque el real no se conoce.
    if double(tick) < fst.tick_prev && ...
       abs(double(encL)) < ag.reset_window && abs(double(encR)) < ag.reset_window
        fst = local_reference(fst, tick, encL, encR);
        fst.t_cum = fst.t_cum + ag.loop_ms;
        fst.n_llc_reset = fst.n_llc_reset + 1;
        fst.still_frames = 0;
        fst.n_rej_consec = 0;
        odom = local_pack(fst, tick, w_m, tilt, imu_ok, ag);
        odom.llc_reset = true;
        emit = true;
        return;
    end

    % --- diferencias contra la ultima trama aceptada ---------------------
    dtick = mod(double(tick) - fst.tick_prev, 2^32);      % [ms], envolvente u32
    dCL   = local_wrap_i32(double(encL) - fst.encL_prev);
    dCR   = local_wrap_i32(double(encR) - fst.encR_prev);

    % El umbral de cuentas es por periodo nominal: un intervalo largo (tramas
    % perdidas en el enlace) admite proporcionalmente mas cuentas.
    n_per = max(1, dtick / ag.loop_ms);
    plausible = dtick > 0 && dtick <= ag.dt_reject*1000 && ...
                abs(dCL) < ag.dcount_reject*n_per && abs(dCR) < ag.dcount_reject*n_per;

    if ~plausible
        fst.n_reject     = fst.n_reject + 1;
        fst.n_rej_consec = fst.n_rej_consec + 1;
        if fst.n_rej_consec >= ag.k_reref
            fst = local_reference(fst, tick, encL, encR);
            fst.t_cum = fst.t_cum + ag.loop_ms;
            fst.n_reref = fst.n_reref + 1;
            fst.n_rej_consec = 0;
            fst.still_frames = 0;
            odom = local_pack(fst, tick, w_m, tilt, imu_ok, ag);
            odom.reref = true;
            emit = true;
        end
        return;
    end
    fst.n_rej_consec = 0;

    % --- conversion fisica (sep_odometry, sin cambios) y acumulacion -----
    [ds, dth, moving] = sep_odometry(dCR, dCL, geo);
    fst.S     = fst.S + ds;
    fst.TH    = fst.TH + dth;
    fst.t_cum = fst.t_cum + dtick;
    fst.n_s   = fst.n_s + 1;
    if imu_ok
        fst.OM  = fst.OM + w_m;
        fst.n_g = fst.n_g + 1;
    end
    if moving > 0.5
        fst.still_frames = 0;
    else
        fst.still_frames = fst.still_frames + 1;
    end

    fst = local_reference(fst, tick, encL, encR);
    odom = local_pack(fst, tick, w_m, tilt, imu_ok, ag);
    emit = true;
end

% =====================================================================
function fst = local_reference(fst, tick, encL, encR)
%#codegen
    fst.tick_prev = double(tick);
    fst.encL_prev = double(encL);
    fst.encR_prev = double(encR);
end

% =====================================================================
function odom = local_pack(fst, tick, w_m, tilt, imu_ok, ag)
%#codegen
    odom = sep_odom_empty();
    odom.n_s          = fst.n_s;
    odom.t_cum        = fst.t_cum;
    odom.tick         = double(tick);
    odom.S            = fst.S;
    odom.TH           = fst.TH;
    odom.OM           = fst.OM;
    odom.n_g          = fst.n_g;
    odom.still_frames = fst.still_frames;
    odom.still        = fst.still_frames >= ag.n_still;
    odom.rollover     = tilt > ag.tilt_rollover;
    odom.imu_ok       = logical(imu_ok);
    odom.w_gyro       = w_m;
    odom.tilt         = tilt;
end

% =====================================================================
function t = local_tilt(acc)
%LOCAL_TILT  Angulo entre el eje z de la IMU y la vertical. Solo detecta
%   vuelco: el acelerometro NO entra al filtro (ver sep_ekf_step).
%#codegen
    a  = double(acc(:));
    an = sqrt(a(1)*a(1) + a(2)*a(2) + a(3)*a(3));
    if an > 0
        t = acos(min(1, max(-1, a(3)/an)));
    else
        t = 0;
    end
end

% =====================================================================
function v = local_wrap_i32(v)
%LOCAL_WRAP_I32  Resta envolvente sobre i32.
%#codegen
    v = mod(v + 2^31, 2^32) - 2^31;
end
