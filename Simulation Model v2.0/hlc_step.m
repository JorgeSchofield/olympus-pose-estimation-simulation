function [st, pose, P, diag_out, lat, publish] = hlc_step(st, buf, n, arrived, ...
                                                          geo, prm, ag, Tb, u_jit)
%HLC_STEP  Un paso base de los agentes del HLC.
%
%   [st, pose, P, diag_out, lat, publish] = HLC_STEP(...)
%
%   Reproduce la arquitectura multiagente sobre CMAES/pthreads
%   (DESIGN.md, ARQ-PE-003):
%
%       adquisicion (por evento, al llegar trama)
%            |  buzon 1, profundidad 1: acumuladores CRUDOS
%       fusion de datos (por evento)          -> sep_fusion_step
%            |  buzon 2, profundidad 1: magnitudes ACUMULADAS (S, TH, OM)
%       estimacion (periodico o por evento, ag.est_mode) -> sep_estimation_step
%            |
%       comunicacion (sigue a estimacion)
%
%   Los dos buzones aplican la misma politica (ag.mbox_mode). Con fusion por
%   evento el buzon 1 casi nunca se llena; los descartes aparecen en el
%   buzon 2 cuando la estimacion es periodica y las tasas se igualan, y la
%   regla de los acumulados los hace inocuos para la distancia.
%
%   ag.test_drop_every > 0 descarta a la fuerza uno de cada N mensajes del
%   buzon 2 (solo para pruebas: test_sep_model, caso 11).
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
    % Si la politica es 'block' y el buzon sigue lleno, el agente de
    % adquisicion esta DETENIDO dentro de MAES_QueueSend. Mientras lo este
    % no drena el puerto serial, asi que el buffer del tty se desborda y
    % las tramas se pierden EN EL ENLACE. Ese es el modo de fallo real del
    % bloqueo: no degrada la latencia, degrada la tasa de entrega, que es
    % un indicador distinto (Objetivo 2, >= 99 %).
    if st.stalled
        if st.box_full
            if arrived
                st.n_stall_loss = st.n_stall_loss + 1;   % trama perdida en el tty
            end
            st.t_stalled = st.t_stalled + Tb;
            st.t = st.t + Tb;
            return;
        else
            st.stalled = false;     % el consumidor vacio el buzon
        end
    end

    if arrived
        [f, ok] = raw_frame_from_bytes(buf, n);
        if ~ok
            st.n_badparse = st.n_badparse + 1;
        else
            st.n_frames = st.n_frames + 1;

            % -------------------------------------------------------------
            % SEMANTICA DEL BUZON. Profundidad 1, como crea CMAES:
            %     xQueueCreate(1, sizeof(MsgObj))
            %
            % CMAES NO tiene una operacion de sobrescritura. Agent_Msg::send
            % es xQueueSend con timeout, de modo que ante buzon lleno:
            %   timeout = 0  -> se descarta el mensaje NUEVO
            %   timeout > 0  -> el emisor se BLOQUEA
            % En ningun caso se reemplaza el mensaje viejo.
            %
            % La diferencia con sobrescribir NO afecta a la distancia: el
            % mensaje lleva acumuladores absolutos y quien diferencia es el
            % consumidor, asi que el desplazamiento de lo descartado sigue
            % contabilizado en el siguiente que si se consuma. Lo que cambia
            % es la EDAD de la muestra que llega al filtro: descartar el
            % nuevo entrega siempre la mas vieja en vez de la mas fresca,
            % del orden de un periodo del consumidor de latencia extra.
            %
            % Se deja como parametro para poder MEDIR la diferencia en vez
            % de suponerla. Por omision, lo que CMAES hace hoy.
            % -------------------------------------------------------------
            acepta = true;
            if st.box_full
                switch ag.mbox_mode
                    case 1      % 'overwrite'
                        % Requiere anadir MAES_QueueOverwrite a la capa de
                        % portabilidad. OJO: CMAES pasa PUNTEROS, no copias,
                        % asi que sobrescribir la ranura no protege el
                        % contenido apuntado. Hace falta ADEMAS la rotacion
                        % de N buffers; sobrescribir a secas introduciria un
                        % uso despues de liberar.
                        st.n_overwrite = st.n_overwrite + 1;
                    case 2      % 'block'
                        st.stalled = true;
                        acepta = false;
                    otherwise   % 0 = 'discard_new', lo que CMAES hace hoy
                        st.n_discard = st.n_discard + 1;
                        acepta = false;
                end
            end

            if acepta
                st.box_tick = f.tick_ms;
                st.box_encL = f.encL;
                st.box_encR = f.encR;
                st.box_gz   = f.gyr(3);
                st.box_acc  = f.acc;
                st.box_tsam = st.t - ag.age_est;  % edad estimada de la muestra
                st.box_full = true;
            end
        end
    end

    % =================================================================
    % AGENTE DE FUSION DE DATOS - por evento
    % =================================================================
    % Consume el buzon 1 en cuanto hay mensaje: su coste es despreciable
    % frente al paso base. Convierte a magnitudes fisicas ACUMULADAS, de modo
    % que un descarte en el buzon 2 tampoco pierde distancia. Con politica
    % 'block' y el buzon 2 lleno, fusion espera (el buzon 1 sigue lleno).
    if st.box_full && ~(ag.mbox_mode == 2 && st.box2_full)
        [st.fst, odom, emit] = sep_fusion_step(st.fst, st.box_tick, st.box_encL, ...
                                               st.box_encR, st.box_gz, st.box_acc, ...
                                               true, geo, ag);
        st.box_full  = false;
        st.n_implaus = st.fst.n_reject;
        if emit
            st.n_odom = st.n_odom + 1;
            if ag.test_drop_every > 0 && mod(st.n_odom, ag.test_drop_every) == 0
                st.n_test_drop = st.n_test_drop + 1;   % descarte forzado (prueba)
            elseif st.box2_full
                if ag.mbox_mode == 1                   % 'overwrite'
                    st.box2_odom = odom;
                    st.box2_tsam = st.box_tsam;
                    st.n_overwrite2 = st.n_overwrite2 + 1;
                else                                   % 'discard_new' (CMAES)
                    st.n_discard2 = st.n_discard2 + 1;
                end
            else
                st.box2_odom = odom;
                st.box2_tsam = st.box_tsam;
                st.box2_full = true;
            end
        end
    end

    % =================================================================
    % AGENTE DE ESTIMACION - periodico (ag.est_mode = 0) o por evento (1)
    % =================================================================
    % Periodico: la tasa efectiva la fija el agente MAS LENTO; si los
    % mensajes llegan mas rapido que este periodo, el buzon 2 descarta.
    % Por evento (DESIGN.md, decision D3): consume el buzon 2 en cuanto hay
    % mensaje, sin el retardo de fase del temporizador.
    run_est = false;
    if ag.est_mode == 1
        run_est = st.box2_full;
    else
        st.wake = st.wake - Tb;
        if st.wake <= 0
            st.wake = st.wake + ag.est_period;
            run_est = st.box2_full;
        end
    end

    if run_est
        [st.est, xe, dg, did] = sep_estimation_step(st.est, st.box2_odom, prm, ag);
        st.box2_full = false;
        st.x = st.est.x;
        st.P = st.est.P;
        if did
            st.n_updates = st.n_updates + 1;
            pose = xe;  P = st.P;  diag_out = dg;
            % AGENTE DE COMUNICACION
            % ag.jitter: los agentes corren sobre pthreads en Linux sin
            % RT-preempt, asi que el despertar no es puntual. El jitter entra
            % directo en la latencia extremo a extremo.
            lat = st.t - st.box2_tsam + ag.est_delay + ag.com_delay ...
                  + ag.jitter*st.u_jit;
            publish = true;
        end
    end

    st.t = st.t + Tb;
end
