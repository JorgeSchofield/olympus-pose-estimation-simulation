function st = channel_init()
%CHANNEL_INIT  Estado inicial del canal: cola de 4 tramas en vuelo.
%#codegen
    st.busy   = zeros(1,4);
    st.rem    = zeros(1,4);
    st.len    = zeros(1,4);
    st.buf    = zeros(4, raw_buf_max(), 'uint8');
    st.n_sent = 0;
    st.n_lost = 0;
end
