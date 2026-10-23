function est = sep_estimation_init(prm)
%SEP_ESTIMATION_INIT  Estado inicial del agente de estimacion.
%
%   est = SEP_ESTIMATION_INIT(prm)
%
%   prm : struct de sep_ekf_params() (usa P0).
%
%   Guarda el estado del EKF y los ultimos acumulados que el agente proceso:
%   las diferencias se calculan contra ESTOS valores, no contra el mensaje
%   anterior del buzon (que pudo perderse). Ver sep_estimation_step.
%
%   Codegen-safe: todos los campos se crean aqui.
%#codegen

    est.x           = zeros(5,1);
    est.P           = diag(prm.P0);
    est.have_ref    = false;
    est.t_prev      = 0;      % [ms] t_cum del ultimo mensaje procesado
    est.S_prev      = 0;
    est.TH_prev     = 0;
    est.OM_prev     = 0;
    est.ng_prev     = 0;
    est.ns_prev     = 0;
    est.n_steps     = 0;      % pasos del agente que actualizaron el filtro
    est.n_substeps  = 0;      % llamadas a sep_ekf_step (>= n_steps)
    est.n_gaps      = 0;      % pasos divididos por intervalo largo
    est.n_rerefs    = 0;      % re-referencias (dt <= 0 o mayor que t_gap_max)
    est.gap         = false;  % el ultimo paso se dividio en subpasos
    est.imu_invalid = false;  % el ultimo paso no tuvo muestras de giroscopio
end
