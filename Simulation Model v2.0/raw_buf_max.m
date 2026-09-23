function n = raw_buf_max()
%RAW_BUF_MAX  Tamano del buffer de trama RAW, en bytes.
%   Mismo valor que raw_buf en main.rs. El peor caso real es 81 bytes
%   (4 + 10 + 6*7 + 2*12 + 1); 100 deja margen.
%#codegen
    n = 100;
end
