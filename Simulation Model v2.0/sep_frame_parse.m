function [s, ok, consumed] = sep_frame_parse(buf)
%SEP_FRAME_PARSE  Busca y decodifica una trama SENSOR en un buffer de bytes.
%
%   [s, ok, consumed] = SEP_FRAME_PARSE(buf)
%
%   Lo ejecuta el AGENTE DE ADQUISICION del HLC. Implementa la maquina de
%   resincronizacion del contrato seccion 3.2.
%
%   ok       : true si se decodifico una trama valida.
%   consumed : bytes que el llamador debe descartar del frente del buffer.
%              Si ok, la trama completa; si no, UN SOLO BYTE.
%
%   POR QUE SE DESCARTA UN SOLO BYTE Y NO LA TRAMA COMPLETA
%   ------------------------------------------------------
%   Ante SOF invalido o CRC fallido no se sabe donde empieza la trama real.
%   Si el fallo fue perdida de sincronismo, el 0xAA 0x55 verdadero puede
%   estar DENTRO de lo que se descartaria. Avanzar un byte a la vez es la
%   unica politica que garantiza encontrarlo.
%
%   REGLA CRITICA DE IMPLEMENTACION (para el port a C)
%   --------------------------------------------------
%   Nunca hacer cast del buffer de recepcion a la estructura. Decodificar
%   campo por campo o con memcpy a una instancia alineada: el acceso por
%   puntero con alineacion insuficiente es comportamiento indefinido en C.
%   Esta funcion decodifica campo por campo a proposito, para que el codigo
%   C que la siga tenga un modelo correcto que copiar.

    SOF0 = 170;  SOF1 = 85;  VER = 1;  TYPE_SENSOR = 1;

    s = sep_frame_empty();
    ok = false;
    consumed = 0;
    n = numel(buf);

    if n < 7,  return; end                    % ni cabecera ni CRC caben

    if buf(1) ~= SOF0 || buf(2) ~= SOF1
        consumed = 1;  return;
    end
    if buf(3) ~= VER || buf(4) ~= TYPE_SENSOR
        consumed = 1;  return;                % CFG u otra version: no aqui
    end

    LEN = buf(5);
    if LEN ~= 48
        consumed = 1;  return;
    end
    total = 5 + LEN + 2;
    if n < total,  return; end                % trama incompleta, esperar

    crc_calc = sep_crc16(buf(3 : 4+LEN));
    crc_rx   = uint16(buf(5+LEN+1) + 256*buf(5+LEN+2));
    if crc_calc ~= crc_rx
        consumed = 1;  return;
    end

    pl = buf(6 : 5+LEN);

    s.t_llc_us = rd_u32(pl, 1);
    for i = 1:6
        s.enc(i) = rd_i32(pl, 5 + 4*(i-1));
    end
    s.seq      = rd_u16(pl, 29);
    s.tx_drop  = rd_u16(pl, 31);
    for i = 1:3
        s.acc(i) = rd_i16(pl, 33 + 2*(i-1));
    end
    s.imu_temp = rd_i16(pl, 39);
    for i = 1:3
        s.gyr(i) = rd_i16(pl, 41 + 2*(i-1));
    end
    s.flags      = pl(47);
    s.stall_mask = pl(48);

    ok = true;
    consumed = total;
end

% ---------------------------------------------------------------------
function v = rd_u32(p, i)
    v = p(i) + 256*p(i+1) + 65536*p(i+2) + 16777216*p(i+3);
end
function v = rd_i32(p, i)
    v = rd_u32(p, i);
    if v >= 2147483648, v = v - 4294967296; end
end
function v = rd_u16(p, i)
    v = p(i) + 256*p(i+1);
end
function v = rd_i16(p, i)
    v = rd_u16(p, i);
    if v >= 32768, v = v - 65536; end
end
