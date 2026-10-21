function [t_ms, gz, acc, imu_ok] = hlc_fusion_inputs(f, t_rx_ms, ag)
%HLC_FUSION_INPUTS  Lo que el agente de fusion recibe de una trama aceptada.
%
%   [t_ms, gz, acc, imu_ok] = HLC_FUSION_INPUTS(f, t_rx_ms, ag)
%
%   f       : trama de raw_frame_from_bytes
%   t_rx_ms : instante de llegada al HLC [ms enteros, reloj del HLC]
%   ag      : sim_params().agents (dt_source, gyro_yaw_axis)
%
%   ESPEJO de pe_fusion_inputs (agents/src/core/pipeline.c), que usan los
%   agentes de adquisicion y fusion de la aplicacion. Asi el modelo y la
%   aplicacion alimentan a sep_fusion_step con los MISMOS valores:
%
%   - base de tiempo (ag.dt_source): 0 = tick del LLC; 1 = llegada al HLC.
%     El firmware actual cuenta el tick por software (+20 por lazo, lazo
%     real ~37 ms), asi que con 0 el filtro ve dt al 53 % del real.
%   - eje de guinada (ag.gyro_yaw_axis, 0 x, 1 y, 2 z): el eje de la IMU que
%     apunta hacia arriba. El mismo eje del acelerometro va en acc(3), que
%     es el que usa la deteccion de vuelco; el orden de los otros dos no
%     importa.
%   - imu_ok: el de la trama (v3.0); una trama v2.20 trae siempre 1.
%#codegen
    if ag.dt_source == 1
        t_ms = mod(t_rx_ms, 2^32);
    else
        t_ms = f.tick_ms;
    end
    a   = ag.gyro_yaw_axis;
    gz  = f.gyr(a + 1);
    acc = [f.acc(mod(a + 1, 3) + 1), f.acc(mod(a + 2, 3) + 1), f.acc(a + 1)];
    imu_ok = f.imu_ok;
end
