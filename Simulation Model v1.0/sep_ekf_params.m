function prm = sep_ekf_params()
%SEP_EKF_PARAMS  Sintonizacion del EKF del SEP (codegen-safe).
%
%   UNICO punto de ajuste del filtro. Nada de esto se replica en otro
%   archivo ni se escribe dentro de un bloque de Simulink.
%
%   El filtro NO usa el GPS. El GPS es exclusivamente verdad de terreno
%   para validacion offline (DRT-SEP-001, PE-CON-001).
%
%   Ref.: DRT-SEP-001 v0.1 seccion 6.2; marco teorico, capitulo de fusion.
%#codegen

    % --- Ruido de proceso -------------------------------------------------
    % Ruido de posicion proporcional a la distancia recorrida:
    % var(ds) = k_rho*|ds| + cuantizacion. Borenstein & Feng (1996) reportan
    % 1-3 % para encoders en superficie plana.
    prm.k_rho = 1e-4;                       % [m] PROV

    % Piso de ruido de ds por cuantizacion del encoder: m_per_tick^2/12.
    % DERIVADO de la geometria; rover_params.m verifica que siga siendo
    % coherente con sep_geo_params y avisa si divergen tras calibrar.
    prm.s2_ds_floor = (4.606e-4)^2 / 12.0;  % [m^2] DER

    % Ruido de integracion del rumbo por paso.
    prm.s2_theta = (0.5*pi/180)^2 * 0.020;  % [rad^2] PROV

    % Varianza de la velocidad angular DERIVADA DE ENCODERS. Este es el
    % parametro que gobierna cuanto confia el filtro en la odometria frente
    % al giroscopio, y es el que se infla al detectar deslizamiento.
    prm.s2_w_enc = (2.0*pi/180)^2;          % [rad^2/s^2] PROV

    % Random walk del sesgo del giroscopio. Debe ser PEQUENO: el sesgo de un
    % MEMS es casi constante en la escala de un ensayo. Un valor grande deja
    % que el sesgo absorba errores de calibracion de la odometria.
    prm.s2_bg = (0.002)^2;                  % [(rad/s)^2/s] PROV

    % --- Ruido de medida --------------------------------------------------
    prm.r_gyro = (0.3*pi/180)^2;            % [rad^2/s^2] PROV
    prm.r_zaru = (0.05*pi/180)^2;           % [rad^2/s^2] PROV - pseudo-medida en reposo

    % --- Deteccion de deslizamiento y adaptacion --------------------------
    % Discrepancia |w_gyro - b_w - w_enc| por encima de la cual se considera
    % patinaje. Identificar con ensayo de rueda elevada (DRT-SEP-001 R-01).
    prm.slip_thresh = 0.10;                 % [rad/s] TBD

    % Ganancia y tope de inflado. Se aplica a s2_w_enc, NO a r_gyro: al
    % patinar hay que desconfiar de los ENCODERS, no del giroscopio.
    % Ver nota de diseno en modelo_sep_ekf.md, seccion 2.3.
    prm.slip_gain = 20.0;                   % [-] PROV
    prm.slip_cap  = 100.0;                  % [-] PROV

    % --- Covarianza inicial -----------------------------------------------
    % El sesgo arranca con incertidumbre PEQUENA a proposito: se deja crecer
    % por random walk y se fija con ZARU. Un P0 grande en el sesgo permite
    % que absorba el error de calibracion en el primer giro.
    prm.P0 = [1e-6; 1e-6; 1e-6; 1e-2; 1e-6];  % diagonal de P0
end
