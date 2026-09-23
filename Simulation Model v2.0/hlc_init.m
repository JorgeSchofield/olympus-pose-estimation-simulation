function st = hlc_init(prm)
%HLC_INIT  Estado inicial de los cuatro agentes del HLC.
%#codegen
    st.x = zeros(5,1);
    st.P = diag(prm.P0);
    st.t = 0;
    st.wake = 0;
    st.u_jit = 0;      % jitter de planificacion, inyectado desde fuera

    st.box_full = false;
    st.box_tick = 0;  st.box_encL = 0;  st.box_encR = 0;
    st.box_gz   = 0;  st.box_tsam = 0;

    st.have_ref = false;
    st.ref_tick = 0;  st.ref_encL = 0;  st.ref_encR = 0;

    st.n_frames   = 0;
    st.n_badparse = 0;
    st.n_implaus  = 0;
    st.n_overwrite= 0;
    st.n_updates  = 0;
end
