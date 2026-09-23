function s = sep_frame_empty()
%SEP_FRAME_EMPTY  Muestra SENSOR en cero, con todos los campos y tamanos.
%   Existe para que empaquetador, parser y emulador compartan una sola
%   definicion de la forma del struct, y para que el parser pueda devolver
%   algo con la forma correcta incluso cuando falla.
%
%   Bits de flags (contrato 3.5, mas dos aditivos de este trabajo):
%     0 ENC_OK        los seis acumuladores se copiaron en seccion critica
%     1 IMU_OK        lectura I2C sin error
%     2 CLK_OK        t_llc_us viene del temporizador, no del contador de ciclos
%     3 OVERRUN       el LLC descarto >=1 muestra
%     4 LLC_SAFE      LLC en Safe o Fault
%     5 TIME_WRAP     t_llc_us dio la vuelta
%     6 ENC_SIDE_SUM  enc(1)=suma derecha, enc(2)=suma izquierda, resto 0
%     7 ROLLOVER      el acelerometro indica que el rover esta volcado
    s.t_llc_us   = 0;
    s.enc        = zeros(1,6);
    s.seq        = 0;
    s.tx_drop    = 0;
    s.acc        = zeros(1,3);
    s.imu_temp   = 0;
    s.gyr        = zeros(1,3);
    s.flags      = 0;
    s.stall_mask = 0;
end
