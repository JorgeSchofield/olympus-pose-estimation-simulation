function frame = sep_frame_pack(s)
%SEP_FRAME_PACK  Serializa una muestra SENSOR a la trama de 55 bytes.
%
%   Implementa el formato congelado del contrato de datos ICD-LLC-002 v1.1,
%   secciones 3.2 y 3.3. Lo ejecuta el EMULADOR DEL LLC: es el lado que en
%   el rover real corre en el ATmega2560.
%
%       off tam campo
%       --- --- -----------------------------------------------------
%         0   1 SOF0 = 0xAA        bit 7 en 1: imposible en ASCII 7 bits
%         1   1 SOF1 = 0x55
%         2   1 VER  = 0x01
%         3   1 TYPE = 0x01 (SENSOR)
%         4   1 LEN  = 48
%         5  48 payload
%        53   2 CRC-16/CCITT-FALSE sobre los bytes [2 .. 4+LEN]
%
%   Payload (48 B), little-endian en todos los campos multi-byte:
%       0  u32    t_llc_us      microsegundos del instante de MUESTREO
%       4  i32[6] enc           acumulados, orden FR FL CR CL RR RL
%      28  u16    seq
%      30  u16    tx_drop
%      32  i16[3] acc           crudo x,y,z   <- reg 0x3B
%      38  i16    imu_temp      crudo         <- reg 0x41
%      40  i16[3] gyr           crudo x,y,z   <- reg 0x43
%      46  u8     flags
%      47  u8     stall_mask
%
%   Los 14 B de los offsets 32-45 son la rafaga I2C 0x3B-0x48 del MPU-9250
%   en su orden nativo, convertida a little-endian y nada mas. La
%   temperatura queda en medio porque ahi la pone el mapa de registros;
%   respetarlo permite auditar la serializacion de un vistazo contra la
%   hoja de datos.
%
%   s : struct con los campos del payload. Los enteros se saturan y
%       envuelven aqui, igual que lo haria el firmware.
%
%   MODO SUMA POR LADO
%   ------------------
%   Cuando el LLC suma las cuentas de sus tres ruedas antes de transmitir,
%   escribe la suma DERECHA en enc(1), la IZQUIERDA en enc(2), deja
%   enc(3..6) en cero y levanta el bit ENC_SIDE_SUM de flags (bit 6, de los
%   reservados). Es un cambio ADITIVO que la politica de §5 permite, y
%   evita el peor modo de fallo posible: que el significado de enc[] cambie
%   en silencio y el HLC lo interprete como seis ruedas casi paradas.

    SOF0 = 170;  SOF1 = 85;  VER = 1;  TYPE_SENSOR = 1;  LEN = 48;

    pl = zeros(1, LEN);

    pl(1:4)   = le_u32(s.t_llc_us);
    for i = 1:6
        pl(5 + 4*(i-1) : 8 + 4*(i-1)) = le_i32(s.enc(i));
    end
    pl(29:30) = le_u16(s.seq);
    pl(31:32) = le_u16(s.tx_drop);
    for i = 1:3
        pl(33 + 2*(i-1) : 34 + 2*(i-1)) = le_i16(s.acc(i));
    end
    pl(39:40) = le_i16(s.imu_temp);
    for i = 1:3
        pl(41 + 2*(i-1) : 42 + 2*(i-1)) = le_i16(s.gyr(i));
    end
    pl(47) = bitand(uint16(s.flags),      255);
    pl(48) = bitand(uint16(s.stall_mask), 255);

    head  = [VER, TYPE_SENSOR, LEN];
    crc   = sep_crc16([head, pl]);           % contrato: bytes [2 .. 4+LEN]
    frame = [SOF0, SOF1, head, pl, le_u16(double(crc))];
end

% ---------------------------------------------------------------------
function b = le_u32(v)
    v = mod(floor(double(v)), 2^32);          % envuelve, no satura
    b = [mod(v,256), mod(floor(v/256),256), ...
         mod(floor(v/65536),256), mod(floor(v/16777216),256)];
end

function b = le_i32(v)
    v = double(v);
    if v < -2147483648, v = -2147483648; end  % satura en los extremos i32
    if v >  2147483647, v =  2147483647; end
    b = le_u32(mod(round(v), 2^32));
end

function b = le_u16(v)
    v = mod(floor(double(v)), 65536);
    b = [mod(v,256), floor(v/256)];
end

function b = le_i16(v)
    v = double(v);
    if v < -32768, v = -32768; end
    if v >  32767, v =  32767; end
    b = le_u16(mod(round(v), 65536));
end
