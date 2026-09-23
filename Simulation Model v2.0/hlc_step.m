function [st, pose, P, diag_out, lat, publish] = hlc_step(st, buf, n, arrived, ...
                                                          geo, prm, ag, Tb, u_jit)
%HLC_STEP  Un paso base de los cuatro agentes del HLC.
%
%   [st, pose, P, diag_out, lat, publish] = HLC_STEP(...)
%
%   Reproduce la arquitectura multiagente sobre CMAES/pthreads:
%
%       adquisicion (por evento, al llegar trama)
%            |  buzon profundidad 1, SOBRESCRIBE
%       estimacion  (periodico, ag.est_period)
%            |
%       comunicacion (sigue a estimacion)
%
%   =====================================================================
%   POR QUE EL BUZON LLEVA ACUMULADORES Y NO INCREMENTOS
%   =====================================================================
%   Los acumuladores absolutos protegen contra la perdida de TRAMA: si una
%   se pierde en el enlace, la siguiente trae el acumulado completo y solo
%   se alarga un dt.
%
%   Pero si el agente de adquisicion convirtiera a INCREMENTOS antes de
%   depositar, esa proteccion se perderia en la frontera siguiente: cuando
%   el buzon de profundidad 1 sobrescribe un mensaje sin consumir, el
%   incremento que llevaba desaparece para siempre y produce error
%   PERMANENTE de posicion. Es el mismo fallo que los acumuladores
%   evitaban, reintroducido un eslabon mas adelante.
%
%   Por eso el buzon guarda (tick, encL, encR) CRUDOS y quien diferencia es
%   el consumidor, contra lo ultimo que el mismo proceso consumio. Con eso
%   la sobrescritura cuesta RESOLUCION TEMPORAL, no distancia.
%
%   REGLA GENERAL: el paso a incrementos va en el ultimo eslabon, nunca
%   antes de un buzon que puede sobrescribir.
%#codegen

    if nargin >= 9
        st.u_jit = u_jit;      % azar inyectado: mantiene identicas las dos rutas
    end

    pose = st.x;
    P    = st.P;
    diag_out = zeros(3,1);
    lat  = 0;
    publish = false;

    % =================================================================
    % AGENTE DE ADQUISICION - por evento
    % =================================================================
    if arrived
        [f, ok] = raw_frame_from_bytes(buf, n);
        if ~ok
            st.n_badparse = st.n_badparse + 1;
        else
            st.n_frames = st.n_frames + 1;
            if st.box_full
                st.n_overwrite = st.n_overwrite + 1;
            end
            st.box_tick = f.tick_ms;
            st.box_encL = f.encL;
            st.box_encR = f.encR;
            st.box_gz   = f.gyr(3);
            st.box_tsam = st.t - ag.age_est;   % edad estimada de la muestra
            st.box_full = true;
        end
    end

    % =================================================================
    % AGENTE DE ESTIMACION - periodico. La tasa efectiva del filtro la fija
    % el agente MAS LENTO, no el mas rapido: si las tramas llegan mas
    % rapido que este periodo, el buzon sobrescribe.
    % =================================================================
    st.wake = st.wake - Tb;
    if st.wake <= 0
        st.wake = st.wake + ag.est_period;

        if st.box_full
            if st.have_ref
                dt  = mod(st.box_tick - st.ref_tick, 2^32) * 1e-3;
                dCL = wrap_i32(st.box_encL - st.ref_encL);
                dCR = wrap_i32(st.box_encR - st.ref_encR);

                % Plausibilidad. La trama ASCII no lleva CRC, de modo que un
                % byte corrompido da un numero creible: esta es la unica
                % defensa disponible en esta ruta.
                % Umbrales del AGENTE, no del filtro: ag.dt_reject y
                % ag.dcount_reject estaban definidos y se ignoraban, con
                % prm.dt_max y un 1e6 escritos a mano en su lugar.
                if dt > 0 && dt <= ag.dt_reject && ...
                   abs(dCL) < ag.dcount_reject && abs(dCR) < ag.dcount_reject
                    [ds, dth, moving] = sep_odometry(dCR, dCL, geo);
                    w_gyro = st.box_gz * ag.gyro_scale;
                    [st.x, st.P, xe, dg] = sep_ekf_step(st.x, st.P, ds, dth, ...
                                                        w_gyro, dt, moving, prm);
                    st.n_updates = st.n_updates + 1;
                    pose = xe;  P = st.P;  diag_out = dg;
                    % AGENTE DE COMUNICACION
                    % ag.jitter: los agentes corren sobre pthreads en Linux
                    % sin RT-preempt, asi que el despertar no es puntual. El
                    % jitter entra directo en la latencia extremo a extremo.
                    lat = st.t - st.box_tsam + ag.est_delay + ag.com_delay ...
                          + ag.jitter*st.u_jit;
                    publish = true;
                else
                    st.n_implaus = st.n_implaus + 1;
                end
            end
            st.ref_tick = st.box_tick;
            st.ref_encL = st.box_encL;
            st.ref_encR = st.box_encR;
            st.have_ref = true;
            st.box_full = false;
        end
    end

    st.t = st.t + Tb;
end

% =====================================================================
function v = wrap_i32(v)
%WRAP_I32  Resta envolvente sobre i32. Sin esto, el paso del acumulador por
%   2^31 se leeria como un teletransporte de 4.29e9 cuentas.
%#codegen
    v = mod(v + 2^31, 2^32) - 2^31;
end
