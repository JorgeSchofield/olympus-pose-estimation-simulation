function [ds, dth, moving, spread] = sep_odometry(enc_delta, geo)
%SEP_ODOMETRY  Odometria skid-steer de 6 ruedas -> diferencial equivalente.
%
%   [ds, dth, moving, spread] = SEP_ODOMETRY(enc_delta, geo)
%
%   Entradas:
%     enc_delta : 6x1, incremento de cuentas desde el paso anterior,
%                 orden FR FL CR CL RR RL, CON SIGNO (cuadratura x2).
%     geo       : struct de sep_geo_params().
%
%   Salidas:
%     ds     : desplazamiento del centro del cuerpo [m]
%     dth    : giro de rumbo por odometria          [rad]
%     moving : 1 si alguna rueda conto en este paso
%     spread : dispersion relativa entre las tres ruedas de un mismo lado
%              [-]. Es un diagnostico de patinaje de rueda individual,
%              complementario al contraste encoder-giroscopio.
%
%   NOTA DE ARQUITECTURA
%   --------------------
%   Esta funcion concentra TODA la dependencia geometrica. El nucleo del
%   EKF (sep_ekf_step) no conoce radios, ticks ni ancho de via: recibe
%   (ds, dth) ya en unidades fisicas. Esa frontera es deliberada y refleja
%   el reparto entre el agente de fusion de datos y el agente de estimacion
%   de la arquitectura multiagente (DRT-SEP-001, PE-RF-006 y PE-RF-007).
%
%   Las cuentas llegan ACUMULADAS por el Canal 2 (ICD-PE-002) y la
%   diferencia se calcula en el HLC. Eso importa: cuantizar cada
%   incremento por separado introduce un sesgo determinista de redondeo,
%   mientras que diferenciar cuentas acumuladas conserva la informacion
%   sub-tick y mantiene el error de cuantizacion acotado.
%#codegen

    % Distancia recorrida por cada rueda. m_per_tick es por rueda, de modo
    % que la dispersion de diametros (error sistematico Ed) queda modelada.
    d = zeros(6,1);
    for i = 1:6
        d(i) = double(enc_delta(i)) * geo.m_per_tick(i);
    end

    % Lado derecho: FR CR RR (indices 1,3,5). Lado izquierdo: FL CL RL (2,4,6).
    dR = (d(1) + d(3) + d(5)) / 3.0;
    dL = (d(2) + d(4) + d(6)) / 3.0;

    ds  = 0.5 * (dR + dL);
    dth = (dR - dL) / geo.B_eff;

    % Movimiento: cualquier rueda con cuentas distintas de cero.
    n = 0;
    for i = 1:6
        n = n + abs(double(enc_delta(i)));
    end
    moving = double(n > geo.stop_ticks);

    % Dispersion intra-lado, normalizada. Si las tres ruedas de un lado
    % deberian recorrer lo mismo y no lo hacen, una esta patinando o
    % bloqueada. Se normaliza por el avance para que sea adimensional.
    ref = max(abs(ds), 1e-6);
    sR = max([abs(d(1)-dR), abs(d(3)-dR), abs(d(5)-dR)]);
    sL = max([abs(d(2)-dL), abs(d(4)-dL), abs(d(6)-dL)]);
    spread = max(sR, sL) / ref;
end
