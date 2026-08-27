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
%   puras asignaciones. Esto no es estilo: el generador de codigo prohibe
%   anadir campos a un struct despues de que el struct ha sido leido, y
%   leer un campo en el lado derecho de una expresion ya cuenta como
%   lectura. Escribir geo.ticks_per_rev = geo.ppr_motor * geo.gear_ratio
%   falla con "addition of new fields after a structure has been read".
%
%   ESTADO DE LOS VALORES: ver p.status en rover_params.m.
%   Ref.: DRT-SEP-001 v0.1, Anexo B.
%#codegen

    % --- Radio por rueda [m], orden FR FL CR CL RR RL ---------------------
    % Vector y no escalar: la dispersion entre ruedas es la fuente del error
    % sistematico Ed (Borenstein & Feng). Ruedas impresas en 3D sin control
    % de tolerancias -> esta dispersion puede no ser despreciable.
    R_wheel = 0.050 * ones(1,6);              % TBD - Anexo B.2.4

    % --- Cadena de conversion de ticks ------------------------------------
    % Decodificacion x2: ISR en ambos flancos de la fase A, fase B leida por
    % GPIO para el signo. NO es x4.
    decode_mode = 2;                          % MED - verificado en main.rs
    ppr_motor   = 11;                         % TBD - no documentado
    gear_ratio  = 31.0;                       % TBD - inferido del nombre del motor

    % Si B.2.1 mide directamente los ticks por vuelta de RUEDA, fijar aqui
    % ese valor: sobrescribe el producto de arriba.
    ticks_per_rev_meas = 0;                   % 0 = todavia no medido

    ticks_per_rev = ppr_motor * gear_ratio * decode_mode;
    if ticks_per_rev_meas > 0
        ticks_per_rev = ticks_per_rev_meas;
    end

    % Metros de avance por tick, por rueda.
    m_per_tick = 2*pi*R_wheel / ticks_per_rev;

    % --- Ancho de via -----------------------------------------------------
    % B_eff NO es el nominal: en un vehiculo de deslizamiento diferencial los
    % centros instantaneos de rotacion se desplazan hacia afuera y el ancho
    % aparente crece. chi = B_eff/B_nom, tipicamente 1.1 a 1.5 segun terreno.
    B_nom = 0.280;                            % TBD - Anexo B.2.5
    chi   = 1.00;                             % TBD - Anexo B.2.6
    B_eff = chi * B_nom;

    % Umbral de "en movimiento": ticks agregados por debajo de los cuales se
    % considera el rover detenido. Habilita la actualizacion ZARU, que es lo
    % unico que hace observable el sesgo del giroscopio.
    stop_ticks = 0;

    % --- Armado del struct: solo asignaciones, ninguna lectura -------------
    geo.R_wheel            = R_wheel;
    geo.decode_mode        = decode_mode;
    geo.ppr_motor          = ppr_motor;
    geo.gear_ratio         = gear_ratio;
    geo.ticks_per_rev_meas = ticks_per_rev_meas;
    geo.ticks_per_rev      = ticks_per_rev;
    geo.m_per_tick         = m_per_tick;
    geo.B_nom              = B_nom;
    geo.chi                = chi;
    geo.B_eff              = B_eff;
    geo.stop_ticks         = stop_ticks;
end