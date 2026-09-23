function [st, buf, n, arrived] = channel_step(st, in_buf, in_n, in_emit, u, ch, Tb)
%CHANNEL_STEP  Un paso base del enlace serial LLC -> HLC.
%
%   [st, buf, n, arrived] = CHANNEL_STEP(st, in_buf, in_n, in_emit, u, ch, Tb)
%
%   Modela el USART0 del ATmega2560 hacia el USB de la RPi5 a 115200 8N1.
%   A nivel de TRAMA y no de byte: el tiempo de un byte son 86.8 us, por
%   debajo del paso base, asi que simular byte a byte multiplicaria el
%   coste sin cambiar ningun resultado.
%
%   u (1x3) : numeros uniformes en [0,1) inyectados desde fuera, para que
%             Simulink y el bucle de MATLAB produzcan lo mismo.
%             u(1) perdida, u(2) corrupcion, u(3) posicion del byte.
%
%   QUE SE MODELA Y POR QUE
%     retardo   : n_bytes*10/baud. Es el suelo de la latencia.
%     p_loss    : trama perdida entera. El indicador admite hasta 1 % de
%                 perdida, asi que el filtro tiene que sobrevivir a que un
%                 dt se duplique. Con acumuladores absolutos el
%                 desplazamiento se recupera integro en la siguiente trama.
%     p_corrupt : un byte alterado. SIN CRC en la trama ASCII, el resultado
%                 es un numero PLAUSIBLE que ningun parser detecta. Es el
%                 riesgo que esta ruta no cubre.
%
%   La cola es de profundidad fija 4: el LLC emite como mucho una trama por
%   ciclo y el retardo de cable es menor que un ciclo, asi que nunca hay
%   mas de una o dos en vuelo. Si se saturara, se descarta la mas vieja y
%   se cuenta como perdida, que es lo que haria un buffer real.
%#codegen

    N = raw_buf_max();
    buf = zeros(1, N, 'uint8');
    n   = 0;
    arrived = false;

    % --- admision de una trama nueva ------------------------------------
    if in_emit
        if u(1) < ch.p_loss
            st.n_lost = st.n_lost + 1;
        else
            b = in_buf;
            if u(2) < ch.p_corrupt
                b = flip_digit(b, in_n, u(3));
            end
            slot = 0;
            for i = 1:4
                if st.busy(i) == 0, slot = i; break; end
            end
            if slot == 0
                st.n_lost = st.n_lost + 1;      % cola llena: se descarta
            else
                st.busy(slot) = 1;
                st.buf(slot,:) = b;
                st.len(slot)   = in_n;
                % Tiempo de cable mas el cuantizado del sondeo del hilo de
                % adquisicion del HLC: no lee cuando llega el ultimo byte,
                % lee cuando despierta.
                st.rem(slot) = double(in_n)*ch.byte_s + ch.poll_s;
            end
            st.n_sent = st.n_sent + 1;
        end
    end

    % --- avance de las tramas en vuelo -----------------------------------
    for i = 1:4
        if st.busy(i) == 1
            st.rem(i) = st.rem(i) - Tb;
            if st.rem(i) <= 0 && ~arrived
                buf = st.buf(i,:);
                n   = st.len(i);
                arrived = true;
                st.busy(i) = 0;
            end
        end
    end
end

% =====================================================================
function b = flip_digit(b, n, u)
%FLIP_DIGIT  Altera un digito. Sin CRC el resultado sigue siendo un numero.
%#codegen
    first = 0;  cnt = 0;
    for i = 1:n
        if b(i) >= uint8('0') && b(i) <= uint8('9')
            cnt = cnt + 1;
            if first == 0, first = i; end
        end
    end
    if cnt == 0, return; end
    target = max(1, min(cnt, ceil(u*cnt)));
    seen = 0;
    for i = 1:n
        if b(i) >= uint8('0') && b(i) <= uint8('9')
            seen = seen + 1;
            if seen == target
                d = double(b(i)) - 48;
                b(i) = uint8(48 + mod(d + 1 + floor(u*8), 10));
                return;
            end
        end
    end
end
