function [st, out] = plant_step(st, v_cmd, w_cmd, slip6, pitch, roll, dt, pl)
%PLANT_STEP  Un paso base de la planta del rover. Fuente unica.
%
%   [st, out] = PLANT_STEP(st, v_cmd, w_cmd, slip6, pitch, roll, dt, pl)
%
%   pl : sub-struct .plant de sim_params(). Se pasa el sub-struct y no la
%        estructura maestra porque rover_params() imprime y avisa, y un
%        bloque MATLAB Function no puede hacer ninguna de las dos cosas.
%
%   La llaman las DOS envolturas: rover_plant (bucle en MATLAB) y el bloque
%   Planta del modelo completo de Simulink. Una sola implementacion.
%
%   ESTADO st
%     .x .y .th        pose verdadera [m, m, rad]
%     .arc (1x6)       arco acumulado por rueda, orden FR FL CR CL RR RL [m]
%     .dist            distancia recorrida [m]
%     .v_prev          velocidad del paso anterior, para la aceleracion
%
%   SALIDA out
%     .pose (1x3) .arc (1x6) .dist .w .a_body (1x2)
%
%   MODELO DE DESLIZAMIENTO
%   Por rueda, con razon s_i en [0,1): la rueda GIRA cubriendo un arco a_i
%   pero el suelo solo avanza a_i*(1-s_i). El encoder mide a_i. Reproduce
%   el punto ciego del sistema: si las seis ruedas tienen el mismo s,
%   encoders y giroscopio concuerdan y ambos mienten.
%
%   CUERPO RIGIDO
%   La rueda en la posicion lateral y_i tiene velocidad longitudinal
%   v - w*y_i. No hay parametro de regimen de acople porque NO ES
%   OBSERVABLE con acumuladores por lado: bajo cinematica rigida la suma de
%   un lado vale 3*(v +- w*h_media), y bajo regimen dominado por motor da
%   la misma expresion. Lo que difiere es el movimiento real a igual
%   lectura, y eso es exactamente lo que parametriza chi.
%#codegen

    % ORIENTACION DE LOS VECTORES. Se fuerza FILA con (:).' antes de operar.
    % No es cosmetico. El bucle de MATLAB pasa slip6 como fila (slip(k,:)),
    % pero Simulink lo entrega como COLUMNA: las senales de un bloque From
    % Workspace son vectores columna. Una fila .* una columna NO da error:
    % la expansion implicita produce una matriz 6x6, mean() devuelve 1x6, y
    % el fallo aparece mucho despues, en ds*cos(thm), como "Incorrect
    % dimensions for matrix multiplication". MATLAB funciona y Simulink no,
    % que es el modo de fallo mas caro de diagnosticar.
    hy  = pl.half_track(:).';
    s6  = slip6(:).';
    y_w = [-hy(1), +hy(1), -hy(2), +hy(2), -hy(3), +hy(3)];

    % --- arco que gira cada rueda (lo que ve el encoder) ---------------
    v_w = v_cmd(1) - w_cmd(1)*y_w;
    a_w = v_w * dt;

    % --- avance real sobre el suelo ------------------------------------
    g_w = a_w .* (1 - s6);

    % --- del suelo al cuerpo: ajuste de (ds, dth) sobre las seis ruedas
    %     g_i = ds - y_i*dth. Con y_w simetrico el sistema se desacopla.
    ds  = mean(g_w);
    dth = -sum(y_w .* (g_w - ds)) / sum(y_w.^2);

    % chi > 1: el ancho de via efectivo del skid-steer amplifica el radio
    % de giro, porque la resistencia lateral disipa parte de la diferencia
    % entre lados en vez de convertirla en rotacion.
    dth = dth / pl.chi_true;

    ds  = ds(1);        % escalares explicitos antes de integrar
    dth = dth(1);

    % --- integracion de la verdad (arco de punto medio) ----------------
    ds  = ds(1);      % blindaje: escalares explicitos antes de integrar
    dth = dth(1);

    thm  = st.th + 0.5*dth;
    st.x = st.x + ds*cos(thm);
    st.y = st.y + ds*sin(thm);
    st.th = wrapPi(st.th + dth);
    st.dist = st.dist + abs(ds);
    st.arc  = st.arc + a_w;

    % --- aceleracion PROPIA en el cuerpo -------------------------------
    % Propia, no fuerza especifica: la gravedad la anade el emulador de la
    % IMU, que es quien conoce la orientacion del chip. Separarlas es lo
    % que evita confundirlas.
    v_now = ds/dt;
    ax = (v_now - st.v_prev)/dt;
    ay = v_now * (dth/dt);                % centripeta
    st.v_prev = v_now;

    out.pose   = [st.x st.y st.th];
    out.arc    = st.arc;
    out.dist   = st.dist;
    out.w      = dth/dt;
    out.a_body = [ax ay];
    out.pitch  = pitch;
    out.roll   = roll;
end

function a = wrapPi(a)
    a = mod(a + pi, 2*pi) - pi;
end
