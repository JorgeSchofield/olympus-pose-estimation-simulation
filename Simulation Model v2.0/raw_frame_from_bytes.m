function [f, ok] = raw_frame_from_bytes(buf, n)
%RAW_FRAME_FROM_BYTES  Parsea una trama RAW recibida como bytes.
%
%   [f, ok] = RAW_FRAME_FROM_BYTES(buf, n)
%
%   Lo ejecuta el agente de adquisicion del HLC. Es el ESPEJO de
%   pe_raw_parse (agents/src/core/raw_parser.c): mismas reglas, de modo que
%   el modelo y la aplicacion aceptan y rechazan exactamente las mismas
%   tramas (agents/tests/test_core.c lo verifica con lineas reales y
%   corruptas). Si se cambia una regla aqui, cambiarla tambien alli.
%
%   Formatos aceptados (el numero de campos decide la version):
%       v2.20  RAW:tick:ax:ay:az:gx:gy:gz:encL:encR                 (9)
%       v3.0   RAW:tick:ax:ay:az:gx:gy:gz:encL:encR:temp:imu_ok     (11)
%   Una trama v2.20 se toma con imu_ok = 1: ese firmware solo emitia RAW
%   cuando la lectura de la IMU tenia exito.
%
%   Reglas: un '\n' y luego un '\r' finales se ignoran; cada campo es un
%   entero decimal de 1 a 11 caracteres con '-' opcional, sin '+' ni
%   espacios; rangos tick u32, IMU y temp i16, encoders i32, imu_ok 0..1;
%   nada despues del ultimo campo.
%
%   NO hay verificacion de integridad: la trama ASCII no lleva CRC, asi que
%   un byte corrompido dentro de un numero pasa este parser. La defensa
%   esta aguas arriba, en el rechazo por plausibilidad de sep_fusion_step.
%#codegen
    f  = struct('tick_ms',0,'acc',zeros(1,3),'gyr',zeros(1,3),'encL',0,'encR',0, ...
                'temp',0,'imu_ok',true,'version',0);
    ok = false;

    len = double(n);
    if len > 0 && buf(len) == uint8(10), len = len - 1; end   % '\n'
    if len > 0 && buf(len) == uint8(13), len = len - 1; end   % '\r'
    if len < 4, return; end
    if ~(buf(1)==uint8('R') && buf(2)==uint8('A') && ...
         buf(3)==uint8('W') && buf(4)==uint8(':'))
        return;
    end

    % el numero de campos decide la version
    nf = 1;
    for k = 5:len
        if buf(k) == uint8(':'), nf = nf + 1; end
    end
    if nf ~= 9 && nf ~= 11, return; end

    lo = [0, -32768*ones(1,6), -2147483648, -2147483648, -32768, 0];
    hi = [4294967295, 32767*ones(1,6), 2147483647, 2147483647, 32767, 1];
    vals = zeros(1,11);
    pos  = 5;
    for fi = 1:nf
        st = pos;
        while pos <= len && buf(pos) ~= uint8(':')
            pos = pos + 1;
        end
        [v, good] = rd_int(buf, st, pos - 1, lo(fi), hi(fi));
        if ~good, return; end
        vals(fi) = v;
        if fi < nf
            pos = pos + 1;               % salta ':' (la cuenta ya se verifico)
        end
    end
    if pos ~= len + 1, return; end

    f.tick_ms = vals(1);
    f.acc     = vals(2:4);
    f.gyr     = vals(5:7);
    f.encL    = vals(8);
    f.encR    = vals(9);
    if nf == 11
        f.temp    = vals(10);
        f.imu_ok  = vals(11) == 1;
        f.version = 3;
    else
        f.temp    = 0;
        f.imu_ok  = true;
        f.version = 2;
    end
    ok = true;
end

% ---------------------------------------------------------------------
function [v, ok] = rd_int(buf, a, b, lo, hi)
%RD_INT  Entero decimal en buf(a:b), como parse_int de raw_parser.c.
    v = 0;  ok = false;
    nc = b - a + 1;
    if nc < 1 || nc > 11, return; end
    i = a;
    neg = false;
    if buf(i) == uint8('-')
        neg = true;
        i = i + 1;
        if nc == 1, return; end
    end
    while i <= b
        c = buf(i);
        if c < uint8('0') || c > uint8('9'), return; end
        v = v*10 + double(c) - 48;
        i = i + 1;
    end
    if neg, v = -v; end
    if v < lo || v > hi, return; end
    ok = true;
end
