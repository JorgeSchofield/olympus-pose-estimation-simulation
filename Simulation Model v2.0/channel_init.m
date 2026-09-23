function st = channel_init(ch)
%CHANNEL_INIT  Estado inicial del canal serial.
%
%   st = CHANNEL_INIT(ch)
%
%   ch : sub-struct .chan de sim_params(). Se necesita porque la
%        profundidad de la cola de tramas en vuelo es un PARAMETRO
%        (ch.max_inflight) y no una constante. Antes estaba fijada a 4 en
%        el codigo mientras el parametro decia 16: el parametro se ignoraba
%        en silencio, que es el peor modo de fallo posible para un ajuste.
%#codegen
    n = ch.max_inflight;
    st.busy   = zeros(1,n);
    st.rem    = zeros(1,n);
    st.len    = zeros(1,n);
    st.buf    = zeros(n, raw_buf_max(), 'uint8');
    st.n_sent = 0;
    st.n_lost = 0;
end
