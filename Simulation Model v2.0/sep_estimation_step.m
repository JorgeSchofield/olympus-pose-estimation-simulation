function [est, x_est, diag_out, did] = sep_estimation_step(est, odom, prm, ag)
%SEP_ESTIMATION_STEP  Un ciclo del agente de estimacion.
%
%   [est, x_est, diag_out, did] = SEP_ESTIMATION_STEP(est, odom, prm, ag)
%
%   Entradas:
%     est  : estado, de sep_estimation_init()
%     odom : mensaje del agente de fusion (sep_odom_empty), ACUMULADO
%     prm  : struct de sep_ekf_params()
%     ag   : parametros de los agentes. Lee: t_gap_max
%
%   Salidas:
%     est      : estado actualizado
%     x_est    : estado del EKF tras el paso
%     diag_out : [slip_metric; q_gain; nis_gyro] del ultimo subpaso
%     did      : true si el filtro avanzo (false en la primera referencia o
%                en una re-referencia)
%
%   QUE HACE
%   --------
%   Diferencia los acumulados contra los ultimos que ESTE agente proceso
%   (no contra el mensaje anterior, que pudo perderse en el buzon):
%
%       dt  = t_cum - t_prev          ds  = S - S_prev
%       dth = TH - TH_prev            z   = (OM - OM_prev) / (n_g - ng_prev)
%
%   y llama a sep_ekf_step SIN CAMBIOS. Sin perdidas, n_g - ng_prev = 1 y la
%   llamada es identica a la que hacia hlc_step con incrementos.
%
%   Con perdidas, el mensaje siguiente trae el avance completo: ds y dth
%   exactos sobre el intervalo, y z es la tasa MEDIA del giroscopio sobre
%   ese mismo intervalo, coherente con w_enc = dth/dt. El ruido de esa media
%   es r_gyro/(n_g - ng_prev); se deja r_gyro, que es conservador en los
%   huecos (raros) y no obliga a tocar sep_ekf_step.
%
%   INTERVALOS LARGOS
%   -----------------
%   sep_ekf_step limita dt a [dt_min, dt_max]. Con acumulados, un intervalo
%   mayor que dt_max lleva un ds EXACTO; recortarlo falsearia w_enc. Por eso
%   se divide en m = ceil(dt/dt_max) subpasos iguales (giro constante en el
%   intervalo), y el recorte de sep_ekf_step queda solo como guarda. Mas alla
%   de ag.t_gap_max se re-referencia sin integrar.
%
%   REPOSO
%   ------
%   La ZARU solo se aplica si TODAS las tramas del paso estuvieron quietas:
%   la racha de reposo (still_frames) debe cubrir las tramas del paso.
%#codegen

    x_est    = est.x;
    diag_out = zeros(3,1);
    did      = false;

    if ~est.have_ref
        est = local_ref(est, odom);
        est.have_ref = true;
        return;
    end

    dt  = (odom.t_cum - est.t_prev) * 1e-3;     % [s]
    ds  = odom.S   - est.S_prev;
    dth = odom.TH  - est.TH_prev;
    dng = odom.n_g - est.ng_prev;
    dns = odom.n_s - est.ns_prev;

    if dt <= 0 || dt > ag.t_gap_max
        est.n_rerefs = est.n_rerefs + 1;
        est.gap = dt > 0;
        est = local_ref(est, odom);
        return;
    end

    % Giroscopio: tasa media sobre el paso. Sin muestras validas se anula la
    % correccion (r_gyro enorme) y el filtro sigue solo con odometria.
    p_use = prm;
    if dng > 0
        z = (odom.OM - est.OM_prev) / dng;
        est.imu_invalid = false;
    else
        z = est.x(4) + est.x(5);
        p_use.r_gyro = 1e12;
        est.imu_invalid = true;
    end

    if odom.still && odom.still_frames >= dns
        moving = 0;
    else
        moving = 1;
    end

    m = max(1, ceil(dt / prm.dt_max - 1e-9));
    for i = 1:m
        [est.x, est.P, x_est, diag_out] = sep_ekf_step(est.x, est.P, ds/m, dth/m, ...
                                                       z, dt/m, moving, p_use);
    end

    est.gap        = m > 1;
    est.n_gaps     = est.n_gaps + double(m > 1);
    est.n_steps    = est.n_steps + 1;
    est.n_substeps = est.n_substeps + m;
    est = local_ref(est, odom);
    did = true;
end

% =====================================================================
function est = local_ref(est, odom)
%#codegen
    est.t_prev  = odom.t_cum;
    est.S_prev  = odom.S;
    est.TH_prev = odom.TH;
    est.OM_prev = odom.OM;
    est.ng_prev = odom.n_g;
    est.ns_prev = odom.n_s;
end
