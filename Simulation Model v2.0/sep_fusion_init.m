function fst = sep_fusion_init()
%SEP_FUSION_INIT  Estado inicial del agente de fusion de datos.
%
%   fst = SEP_FUSION_INIT()
%
%   El agente de fusion recibe los acumuladores CRUDOS del agente de
%   adquisicion y entrega magnitudes fisicas ACUMULADAS (S, TH, OM) al agente
%   de estimacion. Ver sep_fusion_step.
%
%   Codegen-safe: todos los campos se crean aqui y nunca se anaden despues.
%#codegen

    fst.have_ref     = false;
    fst.tick_prev    = 0;   % [ms] tick del LLC de la ultima trama ACEPTADA
    fst.encL_prev    = 0;   % acumuladores crudos de la ultima trama aceptada
    fst.encR_prev    = 0;
    fst.S            = 0;   % [m]     distancia acumulada
    fst.TH           = 0;   % [rad]   giro acumulado de la odometria (sin envolver)
    fst.OM           = 0;   % [rad/s] suma de las tasas del giroscopio
    fst.n_g          = 0;   % muestras sumadas en OM
    fst.n_s          = 0;   % tramas aceptadas
    fst.t_cum        = 0;   % [ms]    tiempo continuo del LLC
    fst.still_frames = 0;   % tramas consecutivas en reposo
    fst.n_rej_consec = 0;   % rechazos consecutivos
    fst.n_reject     = 0;   % tramas rechazadas por plausibilidad
    fst.n_reref      = 0;   % re-referencias tras rechazos consecutivos
    fst.n_llc_reset  = 0;   % reinicios del LLC detectados
end
