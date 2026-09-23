function p = rover_params()
%ROVER_PARAMS  Parametros del modelo de referencia del SEP (rover Olympus).
%
%   Estructura maestra con procedencia y verificacion.
%
%   ARQUITECTURA DE PARAMETROS
%   --------------------------
%   Este archivo NO duplica valores. Envuelve a las funciones que si son la
%   fuente numerica y que ademas son codegen-safe:
%
%       sep_geo_params()  -> geometria y conversion de cuentas
%       sep_ekf_params()  -> sintonizacion del filtro
%       llc_params()      -> modelo temporal y de sensores del LLC
%
%   El codigo generado para el HLC llama a las dos primeras. Este archivo
%   anade lo que solo existe en simulacion (planta, canal, agentes) mas la
%   capa de estado y verificacion, que usa fprintf/warning y por tanto no
%   puede vivir del lado embebido.
%
%   ESTADO DE LOS VALORES (p.status)
%     'MED'  medido y verificado
%     'DER'  derivado de otros parametros (no editar a mano)
%     'PROV' provisional, de campana previa o de hoja de datos
%     'TBD'  pendiente de caracterizacion
%
%   Los resultados obtenidos con cualquier parametro en 'TBD' NO son
%   concluyentes para el indicador de exactitud.

p.meta.version = '0.4';
p.meta.updated = '2026-09-17';
p.meta.llc_ref = 'rover-low-level-controller @ 5a28ff2 (v2.20)';
p.meta.hlc_ref = 'olympus-hlc-rpi5 @ 9f0fb42';
p.status = struct();

% =========================================================================
% 1. GEOMETRIA, FILTRO Y LLC  (fuentes codegen-safe)
% =========================================================================
p.geo = sep_geo_params();
p.ekf = sep_ekf_params();
p.llc = llc_params();

p.status.R_wheel       = 'MED';   % campana 14/09/2026, calibrador
p.status.ticks_per_rev = 'MED';   % campana 14/09/2026 - ver anomalia abajo
p.status.B_nom         = 'MED';   % campana 14/09/2026, cinta metrica
p.status.chi           = 'TBD';   % ensayo de giro de 360 grados
p.status.B_eff         = 'DER';
p.status.m_per_tick    = 'DER';
p.status.k_rho         = 'PROV';
p.status.s2_theta      = 'PROV';
p.status.s2_w_enc      = 'PROV';
p.status.s2_bg         = 'PROV';
p.status.r_gyro        = 'PROV';
p.status.r_zaru        = 'PROV';
p.status.slip_thresh   = 'TBD';   % ensayo de rueda elevada
p.status.slip_gain     = 'PROV';
p.status.s2_ds_floor   = 'DER';

% =========================================================================
% 2-7. PLANTA, IMU, CANAL, AGENTES, ESCENARIO  (fuente: sim_params)
% =========================================================================
sp = sim_params();
p.plant  = sp.plant;
p.drive  = sp.drive;
p.imu    = sp.imu;
p.chan   = sp.chan;
p.agents = sp.agents;
p.sim    = sp.sim;
p.frame  = sp.frame;
p.agents.gyro_lsb = p.llc.gyro_lsb;

p.status.v_max           = 'PROV';  % campana TRL-4; falta reconfirmar en suelo
p.status.w_turn          = 'PROV';
p.status.gyro_bias_z     = 'TBD';   % ensayo en reposo, 60 s - PLANIFICADO
p.status.gyro_bias_drift = 'TBD';   % requiere ensayo termico
p.status.gyro_arw        = 'PROV';  % hoja de datos; confirmar con Allan
p.status.gyro_sf_err     = 'TBD';
p.status.accel_nd        = 'PROV';
p.status.dlpf_cfg        = 'TBD';   % decision abierta, ver llc_params
p.status.half_track      = 'MED';   % campana 14/09/2026 (ext-ext menos 4.5 cm)
p.status.chi_true        = 'TBD';
p.status.slip_model      = 'TBD';
p.status.terrain         = 'TBD';
p.status.p_loss          = 'TBD';
p.status.poll_s          = 'PROV';
p.status.jitter_s        = 'PROV';
p.status.est_period      = 'PROV';
p.status.est_delay       = 'PROV';
p.status.com_delay       = 'PROV';

% =========================================================================
% 8. INDICADORES DEL PROYECTO
% =========================================================================
p.req = sp.req;

% =========================================================================
% 9. VERIFICACION
% =========================================================================
p.check = local_check(p);

end % rover_params


% -------------------------------------------------------------------------
function c = local_check(p)
%LOCAL_CHECK  Coherencia interna e inventario de pendientes.

c.ok = true;
fprintf('\n=== rover_params: verificacion de consistencia ===\n');
fprintf('  LLC ref: %s\n', p.meta.llc_ref);

% --- 1. Temporizacion real del LLC --------------------------------------
% El firmware no tiene reloj: elapsed_ms avanza LOOP_MS fijo mientras
% delay_ms(LOOP_MS) al FINAL del ciclo lo convierte en un hueco, no un
% periodo. El ciclo dura LOOP_MS mas todo el trabajo.
l  = p.llc;
tx = 0;
if l.emit_raw, tx = tx + l.raw_bytes; end
tx = tx + l.tlm_bytes/l.TLM_PERIOD;
work = l.c_misc + l.c_imu + l.c_hc/l.HC_PERIOD + l.c_adc/l.SEN_PERIOD;
if strcmpi(l.tx_mode,'blocking'), work = work + tx*l.byte_s; end

if strcmpi(l.clock_mode,'software')
    cyc = work + l.LOOP_MS*1e-3;
else
    cyc = max(work, l.LOOP_MS*1e-3);
end
c.cycle_s     = cyc;
c.rate_hz     = 1/cyc;
c.clock_scale = (l.LOOP_MS*1e-3)/cyc;

fprintf('  modo de reloj / TX ............ %s / %s\n', l.clock_mode, l.tx_mode);
fprintf('  trabajo por ciclo ............. %.2f ms\n', 1000*work);
fprintf('  ciclo real medio .............. %.2f ms\n', 1000*cyc);
fprintf('  tasa de muestreo real ......... %.1f Hz  (requisito: >= %d Hz)\n', ...
        c.rate_hz, p.req.f_sensor_min);
fprintf('  el reloj del LLC declara el ... %.0f %% del tiempo real\n', 100*c.clock_scale);

if c.rate_hz < p.req.f_sensor_min
    c.ok = false;
    fprintf(['  >> NO CUMPLE la tasa de muestreo. Causa estructural:\n' ...
             '     delay_ms(LOOP_MS) al final del ciclo es un HUECO, no un\n' ...
             '     periodo, asi que el ciclo dura LOOP_MS MAS el trabajo.\n' ...
             '     Se corrige esperando a una fecha limite sobre un timer\n' ...
             '     libre; el trabajo (%.1f ms) cabe en el presupuesto de\n' ...
             '     %.0f ms.\n'], 1000*work, l.LOOP_MS);
end
if c.clock_scale < 0.98
    fprintf(['  >> El dt reportado esta subestimado un %.0f %%. Sesga\n' ...
             '     w_enc = dth/dt hacia arriba y contamina la innovacion\n' ...
             '     del giroscopio en proporcion a la tasa de giro.\n' ...
             '     MEDIBLE SIN INSTRUMENTAR: engine.py sella cada TLM con\n' ...
             '     time.monotonic(); comparar su delta contra el de tick_ms\n' ...
             '     entre TLM consecutivos da este factor directamente.\n'], ...
             100*(1-c.clock_scale));
end

% --- 2. Conversion por lado ----------------------------------------------
% Con acumuladores por lado, la constante es 1/sum_i(N_i/(2 pi R_i)), no el
% promedio de las constantes por rueda. Confundirlas es un error de 3x.
mR = abs(p.geo.m_per_tick_R);
mL = abs(p.geo.m_per_tick_L);
c.asym = mL/mR - 1;
fprintf('  m/cuenta derecho / izquierdo .. %.4e / %.4e\n', mR, mL);
fprintf('  asimetria entre lados ......... %+.2f %%\n', 100*c.asym);
if abs(c.asym) > 0.01
    fprintf(['  NOTA: la asimetria es el error sistematico Ed. El\n' ...
             '        giroscopio la corrige, porque el rumbo ya no sale de\n' ...
             '        la diferencia entre lados. Lo que NO corrige es la\n' ...
             '        escala de DISTANCIA: el ensayo de RECTA mide escala,\n' ...
             '        el cuadrado UMBmark no.\n']);
end

% --- 3. Coherencia del piso de cuantizacion ------------------------------
floor_esp = (0.5*(mR+mL))^2 / 12;
c.floor_ratio = p.ekf.s2_ds_floor / floor_esp;
if c.floor_ratio > 1.5 || c.floor_ratio < 0.67
    c.ok = false;
    warning('rover_params:pisoCuantizacion', ...
       ['prm.s2_ds_floor (%.3e) no corresponde a la geometria actual ' ...
        '(%.3e). Actualizar sep_ekf_params.m tras recalibrar.'], ...
        p.ekf.s2_ds_floor, floor_esp);
end

% --- 4. Anomalia de cuentas por vuelta -----------------------------------
N = mean(p.geo.ticks_per_rev);
R = mean(p.geo.R_wheel);
disp_N = (max(p.geo.ticks_per_rev)-min(p.geo.ticks_per_rev))/N;
c.res_um = 2*pi*R/N * 1e6;
fprintf('  cuentas/vuelta (media) ........ %.0f\n', N);
fprintf('  dispersion entre ruedas ....... %.1f %%\n', 100*disp_N);
fprintf('  resolucion en la llanta ....... %.1f um\n', c.res_um);
if c.res_um < 50 || disp_N > 0.05
    fprintf(['  AVISO: %.0f cuentas/vuelta implican %.1f um de resolucion y\n' ...
             '         una reduccion de ~%.0f:1 con un Hall de 11 PPR y\n' ...
             '         decodificacion x2. Ese motor no tiene esa caja.\n' ...
             '         Junto con el %.1f %% de dispersion entre ruedas (que no\n' ...
             '         tiene explicacion geometrica), la hipotesis que cubre\n' ...
             '         ambas es REBOTE DE FLANCO en el sensor Hall: infla la\n' ...
             '         cuenta y depende del entrehierro, que varia por unidad.\n' ...
             '         CONTRASTE SIN VOLVER AL BANCO: la campana midio a 20,\n' ...
             '         50 y 80 %% de PWM y sep_geo_params promedia las tres.\n' ...
             '         Si las cuentas/vuelta CRECEN con la velocidad, es\n' ...
             '         rebote. Si son planas, la dispersion es real.\n'], ...
             N, c.res_um, N/22, 100*disp_N);
end

% --- 5. Inventario de pendientes -----------------------------------------
f = fieldnames(p.status);
v = struct2cell(p.status);
c.tbd  = f(strcmp(v, 'TBD'));
c.prov = f(strcmp(v, 'PROV'));
fprintf('  parametros TBD ................ %d\n', numel(c.tbd));
fprintf('  parametros provisionales ...... %d\n', numel(c.prov));
if ~isempty(c.tbd)
    c.ok = false;
    fprintf('  pendientes: %s\n', strjoin(c.tbd', ', '));
    fprintf(['  >> Resultados NO concluyentes mientras existan TBD.\n']);
end
fprintf('==================================================\n\n');

end
