function geo = sep_geo_params()
%SEP_GEO_PARAMS  Geometria y conversion de encoders del SEP (codegen-safe).
%
%   Unica fuente numerica de la geometria. rover_params.m la envuelve
%   anadiendo procedencia y verificacion; los bloques de Simulink y el
%   codigo generado llaman a ESTA funcion.
%
%   ESTRUCTURA DEL ARCHIVO
%   ----------------------
%   Todo se calcula en variables LOCALES y el struct se arma al final con
%   puras asignaciones: el generador de codigo prohibe anadir campos a un
%   struct despues de que ha sido leido, y leer un campo en el lado derecho
%   de una expresion ya cuenta como lectura.
%
%   ORIGEN DE LOS VALORES
%   ---------------------
%   Campana de caracterizacion del 14/09/2026. Cada rueda se movio
%   aproximadamente una vuelta por comando, con los demas motores
%   desconectados, a 20 %, 50 % y 80 % de PWM. Diametros y distancias
%   medidos con calibrador y cinta metrica.
%
%   ESTADO DE LOS VALORES: ver p.status en rover_params.m.
%   Ref.: DRT-SEP-001 Anexo B.
%#codegen

    % --- Diametro efectivo por rueda [m], orden FR FL CR CL RR RL ---------
    % Medido: liso 10.3 cm, sobre protuberancias 10.6-10.7 cm, interior
    % 8.2 cm. Las ruedas llevan cinta envolvente que salva los huecos entre
    % protuberancias, de modo que la superficie de rodadura es una
    % circunferencia del diametro de las protuberancias y NO el poligono de
    % doce lados de la llanta impresa. No se aplica correccion poligonal.
    % La medida sobre protuberancias ya incluye el espesor de la cinta.
    %
    % PENDIENTE: medir la distancia recorrida en una vuelta completa sobre
    % el suelo. Es la unica forma directa de obtener el radio efectivo y
    % vuelve innecesaria esta discusion.
    D_efec  = [0.1060 0.1070 0.1070 0.1060 0.1070 0.1070];   % FR FL CR CL RR RL
    R_wheel = 0.5 * D_efec;

    % --- Cuentas por vuelta de rueda, POR RUEDA ---------------------------
    % Medidas, promedio de las tres velocidades. La dispersion entre ruedas
    % es del 24.8 %, sistematica y no aleatoria: FL es la mayor en las tres
    % velocidades y CL/CR las menores. Con diametros practicamente iguales,
    % esa dispersion no tiene explicacion geometrica.
    %
    % PENDIENTE: repetir con 5 o 10 vueltas por rueda. El error de detener
    % "aproximadamente una vuelta" se divide entre el numero de vueltas y
    % permitira distinguir dispersion real de error de medicion.
    ticks_per_rev = [43698 54159 42660 42882 50355 44876];   % FR FL CR CL RR RL

    % --- Signo de los encoders --------------------------------------------
    % Los motores del lado derecho estan montados en espejo, de modo que un
    % avance hacia adelante produce cuentas NEGATIVAS en ese lado y
    % POSITIVAS en el izquierdo. Verificado en la campana: el signo se
    % invierte al comandar reversa, lo que confirma que la cuadratura
    % entrega direccion por hardware.
    %
    % Sin esta correccion, un avance recto se interpreta como giro puro.
    enc_sign = [-1 1 -1 1 -1 1];              % FR FL CR CL RR RL

    % --- Metros de avance por cuenta, con signo ---------------------------
    m_per_tick = enc_sign .* (2*pi*R_wheel ./ ticks_per_rev);

    % --- Ancho de via -----------------------------------------------------
    % Medido de exterior a exterior; el ancho de rueda es 4.5 cm, de modo
    % que centro a centro = medida - 4.5 cm.
    %   FR-FL: 62.6 -> 58.1 cm
    %   CR-CL: 61.6 -> 57.1 cm
    %   RR-RL: 75.9 -> 71.4 cm
    % El eje trasero es 13 cm mas ancho que los otros dos. Corresponde a la
    % geometria real del rocker-bogie: los brazos traseros abren mas que los
    % delanteros y centrales. No es error de medicion.
    %
    % Consecuencia para el modelo: los tres ejes tienen brazos de palanca
    % distintos respecto al centro de rotacion, de modo que el ancho de via
    % efectivo NO es la media aritmetica de los tres. La media se usa solo
    % como valor nominal de partida; el valor que gobierna la cinematica es
    % B_eff, que se identifica experimentalmente.
    %
    % Nota: config.rs declara WHEEL_BASE_MM = 280, menos de la mitad del
    % valor medido. Corregir tambien en el firmware.
    B_nom = 0.622;                            % media de los tres ejes [m]

    % Ancho de via EFECTIVO del modelo skid-steer. chi = B_eff/B_nom se
    % identifica con un ensayo de giro, no con mediciones estaticas.
    chi   = 1.00;                             % TBD - Anexo B.2.6
    B_eff = chi * B_nom;

    % --- Umbral de reposo -------------------------------------------------
    % Con ~46 000 cuentas por vuelta, el ruido de cuantizacion es
    % despreciable y el umbral puede mantenerse en cero.
    stop_ticks = 0;

    % --- Armado del struct: solo asignaciones, ninguna lectura -------------
    geo.R_wheel       = R_wheel;
    geo.ticks_per_rev = ticks_per_rev;
    geo.enc_sign      = enc_sign;
    geo.m_per_tick    = m_per_tick;
    geo.B_nom         = B_nom;
    geo.chi           = chi;
    geo.B_eff         = B_eff;
    geo.stop_ticks    = stop_ticks;
end
