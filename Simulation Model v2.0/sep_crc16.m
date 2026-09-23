function crc = sep_crc16(bytes)
%SEP_CRC16  CRC-16/CCITT-FALSE sobre un vector de bytes.
%
%   Parametros del contrato de datos seccion 3.2: polinomio 0x1021,
%   inicial 0xFFFF, sin reflexion de entrada ni de salida, sin XOR final.
%
%   Se elige sobre un checksum aditivo porque detecta errores de RAFAGA,
%   que es el modo de fallo tipico de un enlace serial con ruido electrico
%   de motores. Un checksum aditivo es ciego a una permutacion de bytes y
%   a dos errores que se compensen.
%
%   bytes : vector de enteros en [0,255] (uint8 o double).
%   crc   : uint16.
%
%   Verificado contra el vector dorado del contrato (CRC = 0x770F) por
%   test_sep_frame.m. Esta implementacion es la TERCERA independiente del
%   proyecto, junto a la de C y la de Python.
%#codegen

    crc = uint16(hex2dec('FFFF'));
    poly = uint16(hex2dec('1021'));
    n = numel(bytes);
    for i = 1:n
        crc = bitxor(crc, bitshift(uint16(bytes(i)), 8));
        for k = 1:8
            if bitand(crc, uint16(hex2dec('8000'))) ~= 0
                crc = bitxor(bitshift(crc, 1), poly);   % bitshift descarta
            else                                        % el bit saliente
                crc = bitshift(crc, 1);
            end
        end
    end
end
