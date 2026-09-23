function test_sep_frame()
%TEST_SEP_FRAME  Prueba de aceptacion del contrato de datos en MATLAB.
%
%   Reproduce el VECTOR DORADO del contrato ICD-LLC-002 v1.1 seccion 3.6.
%   Si esta prueba pasa, el modelo de simulacion es una implementacion
%   independiente del mismo contrato que el firmware Rust y el HLC en C, y
%   puede entrar a CI junto a las otras dos. Si los tres lados se separan,
%   falla aqui antes de que nadie flashee nada.
%
%   Ejecutar:  >> test_sep_frame
    n = 0;

    % --- vector dorado del contrato -------------------------------------
    gold = uint8([ ...
        170  85   1   1  48 205 171   2   1 232   3   0   0  24 252 255 ...
        255  64 226   1   0 192  29 254 255 255 255 255 127   0   0   0 ...
        128 239 190   7   0   0 128   0  64 156 255  46 251 100   0 156 ...
        255 255 127   3  33  15 119]);

    s = sep_frame_empty();
    s.t_llc_us   = 16952269;                 % 0x0102ABCD
    s.enc        = [1000 -1000 123456 -123456 2147483647 -2147483648];
    s.seq        = 48879;                    % 0xBEEF
    s.tx_drop    = 7;
    s.acc        = [-32768 16384 -100];
    s.imu_temp   = -1234;
    s.gyr        = [100 -100 32767];
    s.flags      = 3;
    s.stall_mask = 33;                       % 0x21

    % --- 1) el empaquetador reproduce los 55 bytes exactos --------------
    f = uint8(sep_frame_pack(s));
    assert(numel(f) == 55, 'la trama no mide 55 bytes');
    assert(isequal(f, gold), 'la trama NO coincide con el vector dorado');
    n = n + 1;

    % --- 2) el CRC es 0x770F --------------------------------------------
    crc = sep_crc16(double(gold(3:53)));      % bytes [2 .. 4+LEN]
    assert(crc == uint16(30479), 'CRC incorrecto (esperado 0x770F)');
    n = n + 1;

    % --- 3) ida y vuelta integro ----------------------------------------
    [r, ok, used] = sep_frame_parse(double(gold));
    assert(ok && used == 55, 'el parser no acepto el vector dorado');
    assert(r.t_llc_us == s.t_llc_us       , 'roundtrip: t_llc_us');
    assert(isequal(r.enc, s.enc)          , 'roundtrip: enc (revisar signo i32)');
    assert(r.seq == s.seq && r.tx_drop == s.tx_drop, 'roundtrip: seq/tx_drop');
    assert(isequal(r.acc, s.acc)          , 'roundtrip: acc');
    assert(r.imu_temp == s.imu_temp       , 'roundtrip: imu_temp');
    assert(isequal(r.gyr, s.gyr)          , 'roundtrip: gyr');
    assert(r.flags == s.flags && r.stall_mask == s.stall_mask, 'roundtrip: flags');
    n = n + 1;

    % --- 4) un bit invertido debe fallar el CRC -------------------------
    bad = double(gold);  bad(21) = bitxor(bad(21), 1);
    [~, ok2, used2] = sep_frame_parse(bad);
    assert(~ok2, 'un bit invertido paso el CRC');
    assert(used2 == 1, 'ante CRC fallido debe descartarse UN byte, no la trama');
    n = n + 1;

    % --- 5) resincronizacion: basura delante de una trama valida --------
    %   El relleno incluye un 0xAA suelto a proposito: es el caso que
    %   rompe a un lector que descarte la trama completa en vez de un byte.
    junk = [1 2 170 3 4];
    strm = [junk, double(gold)];
    steps = 0;
    while true
        [r5, ok5, used5] = sep_frame_parse(strm);
        steps = steps + 1;
        if ok5, break; end
        assert(used5 == 1, 'resincronizacion: debe avanzar un byte');
        strm = strm(2:end);
        assert(steps < 64, 'resincronizacion: no encontro la trama');
    end
    assert(steps == numel(junk)+1, 'resincronizacion: numero de pasos inesperado');
    assert(r5.seq == s.seq, 'resincronizacion: trama mal decodificada');
    n = n + 1;

    % --- 6) trama incompleta: esperar, no descartar ---------------------
    [~, ok6, used6] = sep_frame_parse(double(gold(1:40)));
    assert(~ok6 && used6 == 0, 'trama incompleta: no debe consumir nada');
    n = n + 1;

    % --- 7) aritmetica de wrap en los extremos i32 ----------------------
    s7 = s;  s7.enc = [2147483647 -2147483648 0 -1 1 -2147483647];
    r7 = sep_frame_parse(double(sep_frame_pack(s7)));
    assert(isequal(r7.enc, s7.enc), 'wrap i32: signo mal decodificado');
    n = n + 1;

    fprintf('OK: %d pruebas del contrato superadas. Vector dorado reproducido.\n', n);
end
