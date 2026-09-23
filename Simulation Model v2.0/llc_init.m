function st = llc_init()
%LLC_INIT  Estado inicial del emulador del LLC.
%#codegen
    st.phase    = 0;
    st.len      = 1;
    st.k        = 0;
    st.clock_ms = 0;
    st.t_real   = 0;
    st.t_imu    = 0;
    st.t_enc    = 0;
    st.p_imu    = 1;
    st.p_enc    = 1;
    st.gyro_lp  = 0;   % estado del DLPF del giroscopio
    st.lat_gyr  = zeros(1,3);
    st.lat_acc  = zeros(1,3);
    st.lat_cnt  = zeros(1,6);
    st.last_cnt = zeros(1,6);
    st.stall_t  = zeros(1,6);
end
