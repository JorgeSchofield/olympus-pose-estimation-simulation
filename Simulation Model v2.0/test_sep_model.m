function test_sep_model()
%TEST_SEP_MODEL  Pruebas unitarias del modelo. Sin toolboxes.
%   Ejecutar:  >> test_sep_model
    n   = 0;

    % --- 0) contrato de parametros --------------------------------------
    %   Primero, porque si falta un campo todo lo demas falla de forma
    %   confusa a mitad de corrida.
    test_params_contract();

    geo = sep_geo_params();
    prm = sep_ekf_params();
    sp  = sim_params();
    Tb  = sp.sim.Tb;

    % --- 1) constante por lado: NO es la de una rueda -------------------
    %   Es 1/sum_i(N_i/(2*pi*R_i)), aproximadamente un TERCIO de la de una
    %   rueda. Confundirlas es un error de escala de 3x, no un ajuste fino.
    mpt_1 = 2*pi*geo.R_wheel(1)/geo.ticks_per_rev(1);
    razon = mpt_1 / abs(geo.m_per_tick_R);
    assert(razon > 2.5 && razon < 3.5, ...
        'la constante por lado deberia ser ~1/3 de la de una rueda');
    n = n + 1;

    % --- 2) odometria: recta, giro, signo y reposo ----------------------
    %   Cuentas del lado derecho NEGATIVAS al avanzar (montaje en espejo).
    c = 1000;
    [ds, dth, mov] = sep_odometry(-c, c, geo);
    assert(ds > 0 && abs(dth) < 0.02 && mov == 1, 'recta mal resuelta');
    %   Giro en el sitio: cada lado recorre la MISMA distancia en sentidos
    %   opuestos. Con las constantes medidas los dos lados difieren un 3.7 %,
    %   asi que cuentas iguales NO dan distancias iguales: se calculan las
    %   cuentas de cada lado a partir de la distancia. El residuo es de
    %   cuantizacion, menor que una cuenta.
    d = c*abs(geo.m_per_tick_R);
    cR = -round(d/abs(geo.m_per_tick_R));    % derecha adelante (negativo)
    cL = -round(d/abs(geo.m_per_tick_L));    % izquierda atras
    [ds2, dth2, ~] = sep_odometry(cR, cL, geo);
    assert(abs(ds2) < max(abs(geo.m_per_tick_R), abs(geo.m_per_tick_L)), ...
        'giro: ds debe ser ~0 (menos de una cuenta)');
    assert(dth2 > 0, 'giro: derecha adelante debe dar dth>0 (antihorario)');
    [~, ~, mov0] = sep_odometry(0, 0, geo);
    assert(mov0 == 0, 'reposo: moving debe ser 0');
    n = n + 1;

    % --- 3) EKF: guarda de dt -------------------------------------------
    x = zeros(5,1);  P = diag(prm.P0);
    [x1,~,~,~] = sep_ekf_step(x, P, 0, 0, 0, 1e9, 0, prm);
    assert(all(isfinite(x1)), 'dt fuera de rango hizo divergir el filtro');
    n = n + 1;

    % --- 4) ZARU ancla el sesgo -----------------------------------------
    %   En reposo la lectura del giroscopio ES el sesgo. Sin ZARU no hay
    %   nada que lo haga observable.
    bw = 0.012;
    x = zeros(5,1);  P = diag(prm.P0);  P(5,5) = 1e-3;
    for k = 1:4000
        [x, P, ~, ~] = sep_ekf_step(x, P, 0, 0, bw, 0.02, 0, prm);
    end
    assert(abs(x(5) - bw) < 0.2*bw, ...
        'ZARU no convergio: %.5f vs %.5f', x(5), bw);
    n = n + 1;

    % --- 5) el sesgo NO se mueve en movimiento --------------------------
    %   Durante un giro sostenido w y bw no son separables; dejar el sesgo
    %   libre le permite absorber el error de calibracion de la odometria.
    bw0 = x(5);
    for k = 1:500
        [x, P, ~, ~] = sep_ekf_step(x, P, 0.001, 0.002, bw+0.1, 0.02, 1, prm);
    end
    assert(abs(x(5) - bw0) < 1e-6, 'el sesgo se movio en movimiento');
    n = n + 1;

    % --- 6) trama RAW en bytes: ida y vuelta en los extremos ------------
    [buf, nb] = raw_frame_bytes(4294967295, [-32768 16384 -100], ...
                                [100 -100 32767], -2147483648, 2147483647);
    [f, ok] = raw_frame_from_bytes(buf, nb);
    assert(ok, 'el parser rechazo una trama valida');
    assert(f.tick_ms == 4294967295 && isequal(f.acc,[-32768 16384 -100]) && ...
           isequal(f.gyr,[100 -100 32767]) && ...
           f.encL == -2147483648 && f.encR == 2147483647, ...
           'roundtrip RAW: campo mal decodificado');
    assert(all(buf(1:4) == uint8('RAW:')), 'falta el prefijo');
    assert(buf(nb) == uint8(10), 'la trama debe terminar en salto de linea');
    n = n + 1;

    % --- 7) trama RAW: basura rechazada ---------------------------------
    b = uint8('TLM:NORMAL:0');
    [~, o1] = raw_frame_from_bytes(b, numel(b));
    b = uint8('RAW:1:2:3');
    [~, o2] = raw_frame_from_bytes(b, numel(b));
    b = uint8('RAW:1:2:3:4:5:6:7:8:x');
    [~, o3] = raw_frame_from_bytes(b, numel(b));
    assert(~o1 && ~o2 && ~o3, 'el parser acepto una trama invalida');
    n = n + 1;

    % --- 8) el modo de reloj cambia el ciclo, como debe -----------------
    %   Con timer el ciclo es exactamente LOOP_MS; con reloj de software
    %   delay_ms es un HUECO y el ciclo dura LOOP_MS mas el trabajo.
    lp = llc_params();
    lp.clock_mode = 'software';  c1 = cycle_steps(lp, sp, Tb);
    lp.clock_mode = 'timer';     c2 = cycle_steps(lp, sp, Tb);
    assert(c1 > c2, 'el modo software deberia dar un ciclo mas largo');
    assert(c2 == round(lp.LOOP_MS*1e-3/Tb), ...
        'con timer el ciclo deberia ser exactamente LOOP_MS');
    n = n + 1;

    % --- 9) cadena completa: recta, sin ruido ni perdidas ---------------
    %   Con el reloj correcto, un tramo recto debe estimarse con error
    %   pequeno. Es la prueba de humo de que los cuatro pasos encajan.
    sp2 = sp;
    sp2.chan.p_loss = 0;  sp2.chan.p_corrupt = 0;
    sp2.plant.terrain.pitch_amp = 0;  sp2.plant.terrain.roll_amp = 0;
    sp2.plant.chi_true = 1.0;
    imu0 = sp.imu;  imu0.gyro_bias_z = 0;  imu0.gyro_arw = 0;  imu0.accel_nd = 0;
    lp = llc_params();  lp.clock_mode = 'timer';

    stP = plant_init();  stL = llc_init();
    stC = channel_init(sp2.chan); stH = hlc_init(prm);
    N = round(30/Tb);
    for k = 1:N
        [stP, o] = plant_step(stP, 0.029, 0, zeros(1,6), 0, 0, Tb, sp2.plant);
        [stL, b, nb, em] = llc_step(stL, o.w, o.a_body, 0, 0, o.arc, ...
                                    zeros(1,5), lp, geo, imu0, Tb);
        [stC, cb, cn, ar] = channel_step(stC, b, nb, em, [1 1 0], sp2.chan, Tb);
        [stH, ~, ~, ~, ~, ~] = hlc_step(stH, cb, cn, ar, geo, prm, sp2.agents, Tb);
    end
    assert(stH.n_updates > 100, 'la cadena no produjo actualizaciones');
    err = hypot(stH.x(1)-stP.x, stH.x(2)-stP.y);
    assert(err/max(stP.dist,1e-9) < 0.05, ...
        'cadena completa: error de %.1f %% en recta ideal', 100*err/stP.dist);
    n = n + 1;

    % --- 10) una trama perdida NO produce error permanente --------------
    %   Es lo que compran los acumuladores absolutos: solo se alarga un dt.
    stH2 = hlc_init(prm);
    [b1, n1] = raw_frame_bytes(0,    zeros(1,3), zeros(1,3), 0,    0);
    [b2, n2] = raw_frame_bytes(20,   zeros(1,3), zeros(1,3), 1000, -1000);
    [b3, n3] = raw_frame_bytes(40,   zeros(1,3), zeros(1,3), 2000, -2000);
    ag = sp.agents;  ag.est_period = 0.001;
    stA = stH2;
    for b = {{b1,n1},{b2,n2},{b3,n3}}          % todas las tramas
        [stA,~,~,~,~,~] = hlc_step(stA, b{1}{1}, b{1}{2}, true, geo, prm, ag, Tb);
        [stA,~,~,~,~,~] = hlc_step(stA, zeros(1,raw_buf_max(),'uint8'), 0, false, geo, prm, ag, Tb);
    end
    stB = hlc_init(prm);
    for b = {{b1,n1},{b3,n3}}                   % la del medio se pierde
        [stB,~,~,~,~,~] = hlc_step(stB, b{1}{1}, b{1}{2}, true, geo, prm, ag, Tb);
        [stB,~,~,~,~,~] = hlc_step(stB, zeros(1,raw_buf_max(),'uint8'), 0, false, geo, prm, ag, Tb);
    end
    %   La trama perdida solo cambia la resolucion temporal: con las
    %   constantes medidas (3.7 % de asimetria) cada trama lleva un giro
    %   minimo y el orden de integracion deja ~1e-6 m de diferencia. Perder
    %   la DISTANCIA de una trama costaria su avance completo (~2.4 mm), asi
    %   que la cota es el 1 % de ese avance.
    ds_frame = sep_odometry(-1000, 1000, geo);
    dpos = hypot(stA.x(1)-stB.x(1), stA.x(2)-stB.x(2));
    assert(dpos < 0.01*ds_frame, ...
        'una trama perdida produjo error PERMANENTE de posicion (%.3g m)', dpos);
    n = n + 1;

    % --- 11) descartes en el buzon 2 NO pierden distancia ---------------
    %   La regla de los acumuladores aplicada al buzon fusion -> estimacion:
    %   el mensaje lleva S, TH y OM acumulados, asi que descartar uno solo
    %   alarga el paso siguiente. Misma recta ideal que el caso 9, con y sin
    %   un descarte forzado de cada 3 mensajes.
    pose_end = zeros(2,2);
    ndrop = 0;
    for r = 1:2
        ag = sp2.agents;
        if r == 2, ag.test_drop_every = 3; end
        stP = plant_init();  stL = llc_init();
        stC = channel_init(sp2.chan); stH = hlc_init(prm);
        for k = 1:N
            [stP, o] = plant_step(stP, 0.029, 0, zeros(1,6), 0, 0, Tb, sp2.plant);
            [stL, b, nb, em] = llc_step(stL, o.w, o.a_body, 0, 0, o.arc, ...
                                        zeros(1,5), lp, geo, imu0, Tb);
            [stC, cb, cn, ar] = channel_step(stC, b, nb, em, [1 1 0], sp2.chan, Tb);
            [stH, ~, ~, ~, ~, ~] = hlc_step(stH, cb, cn, ar, geo, prm, ag, Tb);
        end
        pose_end(r,:) = stH.x(1:2).';
        if r == 2, ndrop = stH.n_test_drop; end
    end
    assert(ndrop > 100, 'el caso 11 no descarto mensajes (%d)', ndrop);
    dpos = hypot(pose_end(1,1)-pose_end(2,1), pose_end(1,2)-pose_end(2,2));
    assert(dpos < 1e-3, ...
        ['descartar 1 de cada 3 mensajes del buzon 2 movio la pose final ' ...
         '%.2f mm: se esta perdiendo distancia'], 1e3*dpos);
    n = n + 1;

    % --- 12) trama corrupta: la referencia es la ULTIMA ACEPTADA --------
    %   (a) corrupta pero creible: entra, y la trama siguiente la deshace
    %       porque S se recalcula desde acumuladores absolutos.
    %   (b) implausible: se rechaza, y el avance llega integro con la
    %       siguiente trama buena (hlc_step re-referenciaba con la trama
    %       rechazada y perdia distancia).
    ag = sp.agents;
    clean = [0 0 0; 20 1000 -1000; 40 2000 -2000; 60 3000 -3000];
    bad_a = clean;  bad_a(3,2) = 2500;      % 500 cuentas de mas, creible
    bad_b = clean;  bad_b(3,2) = 900000;    % salto imposible
    S = zeros(1,3);  nrej = zeros(1,3);
    seqs = {clean, bad_a, bad_b};
    for r = 1:3
        fs = sep_fusion_init();
        q = seqs{r};
        for k = 1:size(q,1)
            [fs, ~, ~] = sep_fusion_step(fs, q(k,1), q(k,2), q(k,3), 0, ...
                                         [0 0 16384], true, geo, ag);
        end
        S(r) = fs.S;  nrej(r) = fs.n_reject;
    end
    assert(abs(S(2) - S(1)) < 1e-12, 'trama corrupta creible: S no se recupero');
    assert(nrej(3) == 1 && abs(S(3) - S(1)) < 1e-12, ...
        'trama implausible: se perdio distancia o no se rechazo');
    n = n + 1;

    % --- 13) hueco largo: subpasos, no recorte de dt --------------------
    %   2 s de arco (0.058 m, 0.2 rad, giroscopio coherente) en un solo
    %   mensaje. Recortar dt a 0.5 s daria w_enc = 0.4 rad/s contra 0.1 del
    %   giroscopio: una discrepancia falsa que el detector de deslizamiento
    %   tomaria por patinaje. Con subpasos la discrepancia es nula.
    es = sep_estimation_init(prm);
    o0 = sep_odom_empty();
    o1 = o0;
    o1.t_cum = 2000;  o1.S = 0.058;  o1.TH = 0.2;
    o1.n_g = 100;  o1.OM = 100*0.1;  o1.n_s = 100;
    [es, ~, ~, d0] = sep_estimation_step(es, o0, prm, ag);
    [es, xe, dg, d1] = sep_estimation_step(es, o1, prm, ag);
    assert(~d0 && d1, 'la primera llamada debe solo referenciar');
    assert(es.gap && es.n_substeps == 4, 'el hueco de 2 s debia ir en 4 subpasos');
    assert(dg(1) < 1e-9, 'discrepancia giroscopio-odometria falsa: %.3g rad/s', dg(1));
    assert(abs(hypot(xe(1), xe(2)) - 0.058) < 0.01*0.058, ...
        'el hueco largo no conservo la distancia');
    n = n + 1;

    fprintf('OK: %d pruebas superadas.\n', n);
end

% ---------------------------------------------------------------------
function c = cycle_steps(lp, sp, Tb) %#ok<INUSD>
    tx = lp.raw_bytes + lp.tlm_bytes/lp.TLM_PERIOD;
    w  = lp.c_misc + lp.c_imu + lp.c_hc/lp.HC_PERIOD + lp.c_adc/lp.SEN_PERIOD;
    if strcmpi(lp.tx_mode,'blocking'), w = w + tx*lp.byte_s; end
    if strcmpi(lp.clock_mode,'software')
        c = round((w + lp.LOOP_MS*1e-3)/Tb);
    else
        c = round(max(w, lp.LOOP_MS*1e-3)/Tb);
    end
end
