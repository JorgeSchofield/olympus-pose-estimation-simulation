function [f, ok] = raw_frame_from_bytes(buf, n)
%RAW_FRAME_FROM_BYTES  Parsea una trama RAW recibida como bytes.
%
%   [f, ok] = RAW_FRAME_FROM_BYTES(buf, n)
%
%   Lo ejecuta el agente de adquisicion del HLC. Decodifica campo por
%   campo, que es tambien como debe hacerlo el codigo C: nunca hacer cast
%   del buffer de recepcion a una estructura.
%
%   ok = false si falta el prefijo, el numero de campos no es el esperado o
%   algun campo no es un entero decimal. NO hay verificacion de integridad:
%   la trama ASCII del firmware no lleva CRC, asi que un byte corrompido
%   dentro de un numero pasa este parser sin problema. La unica defensa
%   posible esta aguas arriba, en el rechazo por plausibilidad del agente
%   de estimacion.
%#codegen
    f  = struct('tick_ms',0,'acc',zeros(1,3),'gyr',zeros(1,3),'encL',0,'encR',0);
    ok = false;

    if n < 5, return; end
    if ~(buf(1)==uint8('R') && buf(2)==uint8('A') && ...
         buf(3)==uint8('W') && buf(4)==uint8(':'))
        return;
    end

    vals = zeros(1,9);
    pos  = 5;
    for fi = 1:9
        [v, pos, good] = rd_int(buf, pos, n);
        if ~good, return; end
        vals(fi) = v;
        if fi < 9
            if pos > n || buf(pos) ~= uint8(':'), return; end
            pos = pos + 1;
        end
    end

    f.tick_ms = vals(1);
    f.acc     = vals(2:4);
    f.gyr     = vals(5:7);
    f.encL    = vals(8);
    f.encR    = vals(9);
    ok = true;
end

% ---------------------------------------------------------------------
function [v, pos, ok] = rd_int(buf, pos, n)
    v = 0;  ok = false;
    neg = false;
    if pos <= n && buf(pos) == uint8('-')
        neg = true;  pos = pos + 1;
    end
    d = 0;
    while pos <= n && buf(pos) >= uint8('0') && buf(pos) <= uint8('9')
        v = v*10 + double(buf(pos)) - 48;
        pos = pos + 1;
        d = d + 1;
        if d > 11, return; end
    end
    if d == 0, return; end
    if neg, v = -v; end
    ok = true;
end
