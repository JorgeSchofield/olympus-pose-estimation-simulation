function prm = sep_ekf_params()
%SEP_EKF_PARAMS  Sintonizacion del EKF del SEP (codegen-safe).
%
%   UNICO punto de ajuste del filtro. Nada de esto se replica en otro
%   archivo ni se escribe dentro de un bloque de Simulink.
%
%   El filtro NO usa el GPS. El GPS es exclusivamente verdad de terreno
%   para validacion offline.
%#codegen

    % --- Ruido de proceso -------------------------------------------------
    % Ruido de posicion proporcional a la distancia recorrida:
    % var(ds) = k_rho*|ds| + piso. Borenstein y Feng (1996) reportan 1-3 %
    % para encoders en superficie plana.
    prm.k_rho = 1e-4;                       % [m] PROV

    % Piso de ruido de ds por cuantizacion del encoder: m_per_tick^2/12.
    % DERIVADO de la geometria. OJO: con acumuladores POR LADO la constante
    % es 1/sum_i(N_i/(2 pi R_i)), aproximadamente un TERCIO de la de una
    % rueda. rover_params.m verifica que este valor siga siendo coherente
    % con sep_geo_params y avisa si divergen tras recalibrar.
    prm.s2_ds_floor = (2.4e-6)^2 / 12.0;    % [m^2] DER

    % Ruido de integracion del rumbo por paso.
    prm.s2_theta = (0.5*pi/180)^2 * 0.020;  % [rad^2] PROV

    % Varianza de la velocidad angular DERIVADA DE ENCODERS. Gobierna
    % cuanto confia el filtro en la odometria frente al giroscopio, y es el
    % parametro que se infla al detectar deslizamiento.
    prm.s2_w_enc = (2.0*pi/180)^2;          % [rad^2/s^2] PROV

    % Random walk del sesgo del giroscopio. Debe ser PEQUENO: el sesgo de
    % un MEMS es casi constante en la escala de un ensayo. Un valor grande
    % deja que el sesgo absorba errores de calibracion de la odometria.
    prm.s2_bg = (0.002)^2;                  % [(rad/s)^2/s] PROV

    % --- Ruido de medida --------------------------------------------------
    prm.r_gyro = (0.3*pi/180)^2;            % [rad^2/s^2] PROV
    prm.r_zaru = (0.05*pi/180)^2;           % [rad^2/s^2] PROV

    % --- Deteccion de deslizamiento y adaptacion --------------------------
    % Discrepancia |w_gyro - b_w - w_enc| por encima de la cual se considera
    % patinaje. Identificar con ensayo de rueda elevada.
    %
    % ADVERTENCIA DE DISENO: con acumuladores por lado esta es la UNICA
    % senal de deslizamiento disponible. Detecta patinaje ASIMETRICO (un
    % lado mas que el otro). Es ciega al caso en que las seis ruedas
    % patinan por igual, porque entonces encoders y giroscopio concuerdan y
    % ambos mienten. rover_plant.m puede inyectar ese caso por separado
    % para cuantificar cuanto error deja pasar.
    prm.slip_thresh = 0.10;                 % [rad/s] TBD

    % Ganancia y tope de inflado. Se aplica a s2_w_enc, NO a r_gyro: al
    % patinar hay que desconfiar de los ENCODERS, no del giroscopio.
    prm.slip_gain = 20.0;                   % [-] PROV
    prm.slip_cap  = 100.0;                  % [-] PROV

    % --- Plausibilidad de dt ----------------------------------------------
    % El dt llega del reloj del LLC, que en el firmware actual es un
    % contador de software: declara LOOP_MS aunque el ciclo real haya
    % durado mas. Estos limites descartan intervalos imposibles (arranque,
    % vuelta del contador de 32 bits, tramas perdidas en rafaga) sin
    % pretender corregir el sesgo de escala, que no es corregible desde el
    % HLC sin un reloj de referencia.
    prm.dt_min = 0.001;                     % [s]
    prm.dt_max = 0.500;                     % [s]

    % --- Covarianza inicial -----------------------------------------------
    % El sesgo arranca con incertidumbre PEQUENA a proposito: se deja crecer
    % por random walk y se fija con ZARU. Un P0 grande en el sesgo permite
    % que absorba el error de calibracion en el primer giro.
    prm.P0 = [1e-6; 1e-6; 1e-6; 1e-2; 1e-6];
end
