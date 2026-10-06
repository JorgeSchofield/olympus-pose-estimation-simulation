function st = hlc_init(prm)
%HLC_INIT  Estado inicial de los agentes del HLC.
%#codegen
    st.x = zeros(5,1);
    st.P = diag(prm.P0);
    st.t = 0;
    st.wake = 0;
    st.u_jit = 0;      % jitter de planificacion, inyectado desde fuera

    % --- buzon 1: adquisicion -> fusion (acumuladores CRUDOS) -----------
    st.box_full = false;
    st.box_tick = 0;  st.box_encL = 0;  st.box_encR = 0;
    st.box_gz   = 0;  st.box_acc  = zeros(1,3);  st.box_tsam = 0;

    % --- agente de fusion -------------------------------------------------
    st.fst = sep_fusion_init();

    % --- buzon 2: fusion -> estimacion (magnitudes ACUMULADAS) ----------
    st.box2_full = false;
    st.box2_odom = sep_odom_empty();
    st.box2_tsam = 0;

    % --- agente de estimacion --------------------------------------------
    st.est = sep_estimation_init(prm);

    st.n_frames   = 0;
    st.n_badparse = 0;
    st.n_implaus  = 0;     % rechazos por plausibilidad (agente de fusion)
    st.n_overwrite  = 0;   % buzon 1, politica 'overwrite': muere el VIEJO
    st.n_discard    = 0;   % buzon 1, politica 'discard_new': muere el NUEVO
    st.n_stall_loss = 0;   % buzon 1, politica 'block': trama perdida en el tty
    st.t_stalled    = 0;   % [s] tiempo con el agente de adquisicion detenido
    st.stalled      = false;
    st.n_odom       = 0;   % mensajes emitidos por fusion
    st.n_overwrite2 = 0;   % buzon 2: muere el VIEJO
    st.n_discard2   = 0;   % buzon 2: muere el NUEVO
    st.n_test_drop  = 0;   % buzon 2: descartes forzados por ag.test_drop_every
    st.n_updates  = 0;
end
