function [x, P, x_est, diag_out] = sep_ekf_step(x, P, ds, dth_enc, w_gyro, dt, moving, prm)
%SEP_EKF_STEP  Un ciclo del EKF de estimacion de pose del rover Olympus.
%
%   Prediccion con odometria skid-steer por lado, correccion con el
%   giroscopio del MPU-9250. El GPS NO interviene.
%
%   Vector de estado (5x1):
%       x = [ px; py; theta; w; bw ]
%         px,py : posicion en el plano del mundo             [m]
%         theta : rumbo, envuelto a [-pi,pi)                 [rad]
%         w     : velocidad angular de rumbo, fusionada      [rad/s]
%         bw    : sesgo del giroscopio de yaw                [rad/s]
%
%   Entradas:
%       ds      : desplazamiento por odometria en este paso   [m]
%       dth_enc : giro por odometria en este paso             [rad]
%       w_gyro  : tasa de yaw medida por el giroscopio        [rad/s]
%       dt      : intervalo REPORTADO por el LLC              [s]
%       moving  : 1 si los encoders detectan movimiento
%       prm     : struct de sep_ekf_params()
%
%   Salidas:
%       x, P     : estado y covarianza actualizados
%       x_est    : copia de x para registro
%       diag_out : [slip_metric; q_gain; nis_gyro]
%
%   POR QUE 5 ESTADOS
%   -----------------
%   La velocidad la dan los encoders, asi que vx,vy salen del estado y con
%   ellos la restriccion no-holonomica: al modelar el cuerpo como uniciclo,
%   el "no hay velocidad lateral" queda impuesto por la ESTRUCTURA del
%   modelo en lugar de anadirse como pseudo-medida.
%
%   Los sesgos del acelerometro tambien salen, y no por debilidad de
%   observabilidad sino por una razon cuantitativa: a las velocidades de
%   este rover la firma completa de aceleracion propia es ~0.15 m/s^2,
%   mientras que UN GRADO de cabeceo proyecta 0.17 m/s^2 de gravedad sobre
%   el eje longitudinal. La senal esta enterrada bajo la inclinacion, no
%   bajo el ruido. El acelerometro queda fuera del filtro y se usa solo
%   como detector de vuelco.
%
%   OBSERVABILIDAD DEL SESGO DEL GIROSCOPIO
%   ---------------------------------------
%   Durante un giro sostenido, w y bw NO son separables: cualquier
%   discrepancia entre encoders y giroscopio puede explicarse moviendo
%   cualquiera de los dos. Si se deja el sesgo libre, absorbe el error de
%   calibracion del ancho de via y el filtro converge al valor ERRONEO de
%   los encoders, anulando el aporte del giroscopio. Por eso bw solo se
%   actualiza en reposo, donde la velocidad angular verdadera es cero y la
%   lectura del giroscopio ES el sesgo.
%
%   Consecuencia de metodo: los intervalos de reposo son parte del
%   procedimiento de medicion, no una cortesia. Un recorrido sin paradas
%   deja el sesgo sin anclar.
%
%   POR QUE SE ADAPTA Q Y NO R
%   --------------------------
%   Con la odometria en la PREDICCION, la incertidumbre de los encoders
%   vive en el ruido de proceso. Inflar el ruido de MEDIDA durante un
%   deslizamiento degradaria al giroscopio, que es precisamente el sensor
%   del que se depende en ese momento.
%
%   SOBRE dt
%   --------
%   dt es el intervalo que REPORTA el LLC, no el real. En el firmware
%   actual el reloj es un contador de software que declara LOOP_MS aunque
%   el ciclo haya durado mas, de modo que dt viene sistematicamente
%   subestimado. Eso sesga w_enc = dth_enc/dt hacia arriba y contamina la
%   innovacion del giroscopio de forma proporcional a la tasa de giro.
%   El filtro no puede corregirlo sin una referencia externa; el modelo lo
%   reproduce para poder medir cuanto cuesta.
%#codegen

    % ---------------------------------------------------------------------
    % 0) Guarda de dt y deteccion de deslizamiento
    %    Ambas entradas estan disponibles ANTES de predecir, asi que el
    %    inflado de Q se aplica en el mismo paso que lo origina.
    % ---------------------------------------------------------------------
    if dt < prm.dt_min
        dt = prm.dt_min;
    elseif dt > prm.dt_max
        dt = prm.dt_max;
    end

    w_enc = dth_enc / dt;
    slip  = abs(w_gyro - x(5) - w_enc);

    if slip > prm.slip_thresh
        q_gain = 1.0 + prm.slip_gain * (slip/prm.slip_thresh - 1.0);
        if q_gain > prm.slip_cap
            q_gain = prm.slip_cap;
        end
    else
        q_gain = 1.0;
    end

    % ---------------------------------------------------------------------
    % 1) Prediccion con odometria
    % ---------------------------------------------------------------------
    th = x(3);  w = x(4);  bw = x(5);

    thm = th + 0.5*w*dt;            % rumbo en el punto medio del paso
    cm  = cos(thm);
    sm  = sin(thm);

    % Jacobiano de transicion. La fila de w es NULA a proposito: la
    % velocidad angular se reemplaza cada paso por la que dictan los
    % encoders, de modo que no arrastra covarianza previa.
    F = eye(5);
    F(1,3) = -ds*sm;   F(1,4) = -ds*sm*dt*0.5;
    F(2,3) =  ds*cm;   F(2,4) =  ds*cm*dt*0.5;
    F(3,4) =  dt;
    F(4,4) =  0.0;

    xn = [ x(1) + ds*cm;
           x(2) + ds*sm;
           wrap_angle(th + w*dt);
           w_enc;
           bw ];

    % Ruido de proceso. La incertidumbre de ds se proyecta sobre el rumbo
    % actual: el error de odometria es longitudinal, no isotropo.
    s2_ds = prm.k_rho*abs(ds) + prm.s2_ds_floor;

    Q = zeros(5,5);
    Q(1,1) = cm*cm*s2_ds;   Q(1,2) = cm*sm*s2_ds;
    Q(2,1) = Q(1,2);        Q(2,2) = sm*sm*s2_ds;
    Q(3,3) = prm.s2_theta;
    Q(4,4) = prm.s2_w_enc * q_gain;     % <-- knob adaptativo
    Q(5,5) = prm.s2_bg * dt;

    P = F*P*F.' + Q;
    P = 0.5*(P + P.');
    x = xn;

    % ---------------------------------------------------------------------
    % 2) Correccion con el giroscopio
    %    Medida escalar: z = w + bw. No hay inversion de matrices en todo
    %    el filtro, lo que simplifica el port a C embebido.
    % ---------------------------------------------------------------------
    H = zeros(1,5);
    H(4) = 1.0;
    if moving < 0.5
        H(5) = 1.0;     % en reposo el sesgo es observable
    else
        H(5) = 0.0;     % en movimiento se mantiene, no se actualiza
    end

    yk = w_gyro - (x(4) + x(5));
    [x, P, S] = scalar_update(x, P, H, yk, prm.r_gyro);
    nis = yk*yk / S;

    % ---------------------------------------------------------------------
    % 3) ZARU - pseudo-medida de velocidad angular nula en reposo
    %    Es gratis (no requiere sensor) y es lo que ancla el sesgo.
    % ---------------------------------------------------------------------
    if moving < 0.5
        Hz = zeros(1,5);
        Hz(4) = 1.0;
        [x, P, ~] = scalar_update(x, P, Hz, -x(4), prm.r_zaru);
    end

    x_est    = x;
    diag_out = [slip; q_gain; nis];
end

% =====================================================================
function [x, P, S] = scalar_update(x, P, H, yk, R)
% Actualizacion de Kalman para una medida escalar, forma de Joseph.
% Joseph y no (I-KH)P: preserva simetria y semidefinicion positiva bajo
% errores de redondeo, que importa en aritmetica embebida.
%#codegen
    PHt = P*H.';                 % 5x1
    S   = H*PHt + R;             % escalar
    K   = PHt / S;               % 5x1

    x    = x + K*yk;
    x(3) = wrap_angle(x(3));

    I  = eye(5);
    IK = I - K*H;
    P  = IK*P*IK.' + (K*K.')*R;
    P  = 0.5*(P + P.');
end

% =====================================================================
function a = wrap_angle(a)
% Envuelve un angulo al intervalo [-pi, pi).
%#codegen
    a = mod(a + pi, 2*pi) - pi;
end
