function [ds, dth, moving] = sep_odometry(dC_R, dC_L, geo)
%SEP_ODOMETRY  Odometria skid-steer por lado -> diferencial equivalente.
%
%   [ds, dth, moving] = SEP_ODOMETRY(dC_R, dC_L, geo)
%
%   v0.3 - Entradas POR LADO. El LLC suma las cuentas de sus tres ruedas
%   antes de transmitir, de modo que el HLC recibe dos acumuladores y no
%   seis. Ver la derivacion de la constante por lado en sep_geo_params.
%
%   Entradas:
%     dC_R : incremento del acumulador del lado DERECHO  [cuentas]
%     dC_L : incremento del acumulador del lado IZQUIERDO [cuentas]
%
%            CONVENCION DE SIGNO. Los motores del lado derecho estan
%            montados en espejo: al avanzar, el lado izquierdo cuenta
%            POSITIVO y el derecho NEGATIVO. Esta funcion no rectifica el
%            signo aqui, porque geo.m_per_tick_R ya lo lleva incorporado.
%            Pasar cuentas ya rectificadas produce un doble cambio de signo
%            y un avance recto se leeria como giro puro.
%
%     geo  : struct de sep_geo_params().
%
%   Salidas:
%     ds     : desplazamiento del centro del cuerpo [m]
%     dth    : giro de rumbo por odometria          [rad]
%     moving : 1 si algun lado conto en este paso
%
%   NOTA DE ARQUITECTURA
%   --------------------
%   Esta funcion concentra TODA la dependencia geometrica. El nucleo del
%   EKF (sep_ekf_step) no conoce radios, ticks ni ancho de via: recibe
%   (ds, dth) ya en unidades fisicas. Esa frontera refleja el reparto entre
%   el agente de fusion de datos y el agente de estimacion de la
%   arquitectura multiagente (DRT-SEP-001, PE-RF-006 y PE-RF-007).
%
%   ACUMULADORES, NO INCREMENTOS
%   ----------------------------
%   Las cuentas llegan ACUMULADAS y la diferencia se calcula en el HLC con
%   resta envolvente sobre i32. Importa por dos razones:
%     1) Cuantizar cada incremento por separado introduce un sesgo
%        determinista de redondeo; diferenciar acumulados conserva la
%        informacion sub-tick y acota el error de cuantizacion.
%     2) Una trama perdida solo alarga un dt; el desplazamiento se recupera
%        integro en la siguiente. Con incrementos seria error PERMANENTE de
%        posicion, inaceptable con hasta 1 % de perdida admitida.
%
%   QUE SE PERDIO AL SUMAR POR LADO
%   -------------------------------
%   La version anterior devolvia ademas la dispersion intra-lado, que
%   detectaba una rueda individual patinando o bloqueada sin sensores
%   adicionales. Con acumuladores por lado esa senal no existe: tres ruedas
%   sumadas son un solo numero. Queda UNA sola senal de deslizamiento, la
%   discrepancia giroscopio-encoders, con su punto ciego declarado: si las
%   seis ruedas patinan por igual, ambos sensores concuerdan y el filtro
%   integra distancia que no ocurrio.
%   La dispersion se recupera sin tocar el contrato si alguna vez el LLC
%   vuelve a transmitir las seis cuentas; el campo enc[] de la trama tiene
%   los seis huecos reservados.
%#codegen

    % Distancia recorrida por cada lado. La constante lleva el signo del
    % montaje en espejo (ver sep_geo_params).
    dR = double(dC_R) * geo.m_per_tick_R;
    dL = double(dC_L) * geo.m_per_tick_L;

    ds  = 0.5 * (dR + dL);
    dth = (dR - dL) / geo.B_eff;

    % Reposo: habilita ZARU, que es lo unico que hace observable el sesgo
    % del giroscopio. Se mide sobre las cuentas crudas, no sobre metros,
    % para que el umbral se exprese en la unidad que produce el sensor.
    n = abs(double(dC_R)) + abs(double(dC_L));
    moving = double(n > geo.stop_ticks);
end
