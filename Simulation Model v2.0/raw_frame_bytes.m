function [buf, n] = raw_frame_bytes(tick_ms, acc, gyr, encL, encR)
%RAW_FRAME_BYTES  Trama RAW como vector de bytes uint8, igual que el LLC.
%
%   [buf, n] = RAW_FRAME_BYTES(tick_ms, acc, gyr, encL, encR)
%
%   buf : uint8(1 x RAW_BUF_MAX), relleno con ceros a partir de n
%   n   : bytes validos
%
%   Los bytes SON los caracteres ASCII de
%
%       RAW:<tick>:<ax>:<ay>:<az>:<gx>:<gy>:<gz>:<encL>:<encR>\n
%
%   generados con la misma conversion entero-a-decimal del firmware
%   (write_u32 y write_i32 de main.rs): sin relleno, sin signo de mas, y el
%   negativo como un guion seguido de la magnitud. Por eso la LONGITUD ES
%   VARIABLE -- entre ~40 y ~81 bytes segun los valores -- y por eso el
%   tiempo de transmision tambien lo es. Un modelo que asumiera trama de
%   tamano fijo perderia esa variabilidad, que a 115200 y con transmision
%   bloqueante se paga en tiempo de lazo.
%
%   Se trabaja en bytes y no en texto para poder inyectar corrupcion de un
%   byte: la trama ASCII del firmware NO lleva CRC, de modo que un byte
%   alterado produce un numero PLAUSIBLE que nadie detecta. Ese es el
%   riesgo que esta ruta no cubre y que el formato binario con CRC si.
%
%   Tamano de buffer: el peor caso es 4 + 10 + 6*7 + 2*12 + 1 = 81 bytes.
%   RAW_BUF_MAX = 100 deja margen y es el mismo valor que reserva el
%   firmware para raw_buf.
%#codegen
    N   = raw_buf_max();
    buf = zeros(1, N, 'uint8');
    i   = 0;

    pre = uint8('RAW:');
    buf(1:4) = pre;  i = 4;

    [buf, i] = wr_u32(tick_ms, buf, i);
    for k = 1:3
        i = i + 1;  buf(i) = uint8(':');
        [buf, i] = wr_i32(acc(k), buf, i);
    end
    for k = 1:3
        i = i + 1;  buf(i) = uint8(':');
        [buf, i] = wr_i32(gyr(k), buf, i);
    end
    i = i + 1;  buf(i) = uint8(':');
    [buf, i] = wr_i32(encL, buf, i);
    i = i + 1;  buf(i) = uint8(':');
    [buf, i] = wr_i32(encR, buf, i);
    i = i + 1;  buf(i) = uint8(10);          % '\n'

    n = i;
end

% ---------------------------------------------------------------------
function [buf, i] = wr_u32(v, buf, i)
% Equivalente de write_u32 del firmware: digitos decimales, sin relleno.
    v = mod(floor(double(v)), 2^32);
    if v == 0
        i = i + 1;  buf(i) = uint8('0');  return;
    end
    tmp = zeros(1,10);
    len = 0;
    while v > 0
        len = len + 1;
        tmp(len) = mod(v, 10);
        v = floor(v/10);
    end
    for k = len:-1:1
        i = i + 1;  buf(i) = uint8(48 + tmp(k));
    end
end

% ---------------------------------------------------------------------
function [buf, i] = wr_i32(v, buf, i)
% Equivalente de write_i32: guion y magnitud. Nota sobre i32::MIN: el
% firmware hace val.wrapping_neg() as u32, que para -2147483648 da
% 2147483648, de modo que imprime -2147483648 correctamente.
    v = double(v);
    if v < 0
        i = i + 1;  buf(i) = uint8('-');
        [buf, i] = wr_u32(-v, buf, i);
    else
        [buf, i] = wr_u32(v, buf, i);
    end
end
