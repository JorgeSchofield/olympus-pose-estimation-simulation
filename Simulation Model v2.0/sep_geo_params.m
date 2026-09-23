function geo = sep_geo_params()
%SEP_GEO_PARAMS  Geometria y conversion de encoders del SEP (codegen-safe).
%
%   v0.3 - Acumuladores POR LADO. El LLC entrega dos sumas de cuentas, no
%   seis contadores individuales. Esto cambia la conversion de cuentas a
%   metros de forma NO trivial; ver la derivacion abajo.
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
%   =====================================================================
%   DERIVACION DE LA CONSTANTE POR LADO
%   =====================================================================
%   Las tres ruedas de un lado recorren aproximadamente la misma distancia
%   d sobre el suelo, pero cada una tiene su propio radio R_i y sus propias
%   cuentas por vuelta N_i. La rueda i aporta entonces
%
%       c_i = d * N_i / (2*pi*R_i)                      [cuentas]
%
%   y el acumulador del lado, que es la SUMA de las tres, vale
%
%       C = sum_i c_i = d * sum_i N_i/(2*pi*R_i)
%
%   de donde la constante de conversion del lado es el INVERSO de esa suma:
%
%       m_per_tick_lado = 1 / sum_i ( N_i / (2*pi*R_i) )
%
%   CONSECUENCIA IMPORTANTE: no es el promedio de las constantes por rueda,
%   ni la constante de una rueda representativa. Es aproximadamente un
%   TERCIO de la constante de una sola rueda. Usar por descuido el valor de
%   una rueda produciria un error de escala de 3x, no un ajuste fino.
%
%   Con los valores de la campana del 14/09/2026 los dos lados difieren un
%   3.7 %. Esa asimetria es el error sistematico Ed de Borenstein y Feng:
%   sin calibrar, una recta de 2 m acumula ~6.9 deg de desvio de rumbo. El
%   giroscopio la corrige, porque el rumbo ya no sale de la diferencia
%   entre lados. Lo que el giroscopio NO corrige es la escala de DISTANCIA:
%   si ambos lados estan mal un 1.8 %, la distancia esta mal un 1.8 %.
%   De ahi la prioridad de calibracion: el ensayo de RECTA mide escala, el
%   cuadrado UMBmark no.
%
%   =====================================================================
%   POR QUE B_nom ES LA MEDIA ARITMETICA (y antes no lo era)
%   =====================================================================
%   Los tres ejes tienen anchos distintos: 58.1, 57.1 y 71.4 cm de centro a
%   centro (el trasero abre mas, es geometria real del rocker-bogie, no
%   error de medicion). Con odometria por rueda habria que llevar la
%   semivia de cada eje por separado. Al SUMAR las tres ruedas de un lado,
%   la suma de los tres brazos de palanca da exactamente tres veces la
%   media, de modo que el ancho de via que gobierna la formula por lado ES
%   la media aritmetica:
%
%       (0.2905 + 0.2855 + 0.3570)/3 = 0.3110 = B_nom/2
%
%   Esto tambien vuelve irrelevante la pregunta de si las ruedas siguen la
%   cinematica rigida o la velocidad que les impone el motor: cualquier
%   diferencia queda absorbida en chi, que se identifica con un ensayo de
%   giro y no con mediciones estaticas.
%
%   ESTADO DE LOS VALORES: ver p.status en rover_params.m.
%   Ref.: DRT-SEP-001 Anexo B. Contrato de datos ICD-LLC-002 v1.1.
%#codegen

    % --- Diametro efectivo por rueda [m], orden FR FL CR CL RR RL ---------
    % Medido: liso 10.3 cm, sobre protuberancias 10.6-10.7 cm. Las ruedas
    % llevan cinta envolvente que salva los huecos entre protuberancias, de
    % modo que la superficie de rodadura es una circunferencia del diametro
    % de las protuberancias y NO el poligono de doce lados de la llanta
    % impresa. La medida ya incluye el espesor de la cinta.
    %
    % PENDIENTE: medir la distancia recorrida en una vuelta completa sobre
    % el suelo. Es la unica forma directa de obtener el radio efectivo.
    D_efec  = [0.1060 0.1070 0.1070 0.1060 0.1070 0.1070];   % FR FL CR CL RR RL
    R_wheel = 0.5 * D_efec;

    % --- Cuentas por vuelta de rueda, POR RUEDA ---------------------------
    % Medidas, promedio de las tres velocidades de ensayo.
    %
    % ANOMALIA ABIERTA (dos, en realidad):
    %  1) La dispersion entre ruedas es del 24.8 %, sistematica: FL es la
    %     mayor en las tres velocidades y CL/CR las menores. Las cuentas por
    %     vuelta dependen del encoder y de la reduccion, NO del diametro, de
    %     modo que no tiene explicacion geometrica.
    %  2) El valor medio, ~46 400 cuentas por vuelta de RUEDA, implica 7.2 um
    %     de resolucion en la llanta y una reduccion de 2111:1 con un Hall de
    %     11 PPR y decodificacion x2. Ese motor no tiene esa caja.
    %
    % HIPOTESIS QUE EXPLICA LAS DOS A LA VEZ: rebote de flanco en el sensor
    % Hall. Infla la cuenta, y depende del entrehierro iman-sensor, que
    % varia de unidad a unidad. CONTRASTE SIN VOLVER AL BANCO: la campana
    % midio a 20, 50 y 80 % de PWM y este archivo promedia las tres. Si las
    % cuentas por vuelta CRECEN con la velocidad, es rebote. Si son planas,
    % la dispersion es real. Mirar los tres numeros antes de promediarlos.
    ticks_per_rev = [43698 54159 42660 42882 50355 44876];   % FR FL CR CL RR RL

    % --- Indices de lado --------------------------------------------------
    % Orden normativo del contrato 3.3: FR FL CR CL RR RL. LOS LADOS NO SON
    % CONTIGUOS. Se usan como constantes nombradas, nunca como rangos.
    idx_R = [1 3 5];    % FR CR RR
    idx_L = [2 4 6];    % FL CL RL

    % --- Signo de los encoders --------------------------------------------
    % Los motores del lado derecho estan montados en espejo: un avance hacia
    % adelante produce cuentas NEGATIVAS en ese lado y POSITIVAS en el
    % izquierdo. Verificado en la campana (el signo se invierte al comandar
    % reversa, lo que confirma que la cuadratura entrega direccion por
    % hardware). Sin esta correccion, un avance recto se lee como giro puro.
    %
    % Al sumar por lado el signo es POR LADO, no por rueda.
    sign_R = -1;
    sign_L = +1;

    % --- Metros por cuenta, POR LADO (ver derivacion en la cabecera) ------
    sum_R = 0;
    sum_L = 0;
    for k = 1:3
        iR = idx_R(k);
        iL = idx_L(k);
        sum_R = sum_R + ticks_per_rev(iR) / (2*pi*R_wheel(iR));
        sum_L = sum_L + ticks_per_rev(iL) / (2*pi*R_wheel(iL));
    end
    m_per_tick_R = sign_R / sum_R;      % [m/cuenta] lleva el signo del lado
    m_per_tick_L = sign_L / sum_L;

    % --- Ancho de via -----------------------------------------------------
    % Medido de exterior a exterior; ancho de rueda 4.5 cm, centro a centro
    % = medida - 4.5 cm.
    %   FR-FL: 62.6 -> 58.1 cm      semivia 0.2905
    %   CR-CL: 61.6 -> 57.1 cm      semivia 0.2855
    %   RR-RL: 75.9 -> 71.4 cm      semivia 0.3570
    % Media de los tres ejes. Con acumuladores por lado esta media es la
    % cifra correcta, no una aproximacion (ver cabecera).
    %
    % Nota: config.rs declara WHEEL_BASE_MM = 280, menos de la mitad del
    % valor medido. Corregir tambien en el firmware.
    B_nom = 0.622;                            % [m]

    % Ancho de via EFECTIVO del modelo skid-steer. chi = B_eff/B_nom se
    % identifica con un ensayo de giro, no con mediciones estaticas.
    chi   = 1.00;                             % TBD - Anexo B.2.6
    B_eff = chi * B_nom;

    % --- Umbral de reposo -------------------------------------------------
    % Cuentas totales (ambos lados) por debajo de las cuales se considera
    % que el rover no se movio. Habilita ZARU, que es lo unico que hace
    % observable el sesgo del giroscopio. Con ~46 000 cuentas por vuelta el
    % ruido de cuantizacion es despreciable y el umbral puede ser cero; si
    % la anomalia de cuentas por vuelta se resuelve a la baja, hay que
    % subirlo o el rover nunca se declarara en reposo.
    stop_ticks = 0;

    % --- Armado del struct: solo asignaciones, ninguna lectura -------------
    geo.R_wheel       = R_wheel;
    geo.ticks_per_rev = ticks_per_rev;
    geo.idx_R         = idx_R;
    geo.idx_L         = idx_L;
    geo.sign_R        = sign_R;
    geo.sign_L        = sign_L;
    geo.m_per_tick_R  = m_per_tick_R;
    geo.m_per_tick_L  = m_per_tick_L;
    geo.B_nom         = B_nom;
    geo.chi           = chi;
    geo.B_eff         = B_eff;
    geo.stop_ticks    = stop_ticks;
end
