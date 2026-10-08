function codegen_hlc_core(outdir)
%CODEGEN_HLC_CORE  Genera en C el nucleo de los agentes de fusion y estimacion.
%
%   CODEGEN_HLC_CORE()        genera en ./codegen/hlc_core
%   CODEGEN_HLC_CORE(outdir)  genera en outdir, por ejemplo la carpeta
%                             agents/src/core/gen de la aplicacion CMAES
%
%   Requiere MATLAB Coder. Genera una biblioteca estatica (solo codigo, sin
%   compilar) con cuatro puntos de entrada:
%
%       sep_fusion_init      sep_fusion_step       -> agente de fusion
%       sep_estimation_init  sep_estimation_step   -> agente de estimacion
%
%   sep_odometry y sep_ekf_step entran como funciones internas, SIN CAMBIOS.
%
%   REGLAS DEL CODIGO GENERADO (DESIGN.md, D14 y PE-RNF-007)
%   ---------------------------------------------------------
%   - C89/C90 (compila como C99), sin memoria dinamica, para ARM 64 bits.
%   - Sin estado persistente: todo el estado vive en fst y est, que son de
%     los agentes. El codigo generado es reentrante.
%   - Los parametros son ARGUMENTOS, no constantes plegadas: la aplicacion
%     llena geo, prm y ag desde pose.conf al arrancar, asi que recalibrar no
%     obliga a regenerar ni a recompilar (ver export_pose_conf).
%
%   QUE STRUCTS RECIBE
%   ------------------
%   Solo los campos que el codigo lee, para que los structs de C sean
%   pequenos y no arrastren texto (sim_params().agents lleva cadenas):
%     geo : m_per_tick_R, m_per_tick_L, B_eff, stop_ticks
%     ag  : gyro_scale, dt_reject, dcount_reject, loop_ms, reset_window,
%           k_reref, n_still, tilt_rollover, t_gap_max
%     prm : sep_ekf_params() completo
%   Si sep_fusion_step o sep_estimation_step empiezan a leer otro campo, la
%   generacion falla aqui con "campo inexistente": anadirlo en local_ag o
%   local_geo y en export_pose_conf.
%
%   Despues de generar, ejecutar test_sep_model: el C generado se compara
%   contra estas mismas funciones en las pruebas de la aplicacion
%   (DESIGN.md, V-U-3 y V-U-5).

    if nargin < 1 || isempty(outdir)
        outdir = fullfile(fileparts(mfilename('fullpath')), 'codegen', 'hlc_core');
    end

    sp  = sim_params();
    geo = local_geo(sep_geo_params());
    ag  = local_ag(sp.agents);
    prm = sep_ekf_params();

    fst  = sep_fusion_init();
    est  = sep_estimation_init(prm);
    odom = sep_odom_empty();

    % Nombres estables para los structs de C (si no, Coder los llama
    % struct0_T, struct1_T... y cambian al tocar cualquier campo).
    geoT  = coder.cstructname(coder.typeof(geo),  'sep_geo_t');
    agT   = coder.cstructname(coder.typeof(ag),   'sep_agent_params_t');
    prmT  = coder.cstructname(coder.typeof(prm),  'sep_ekf_params_t');
    fstT  = coder.cstructname(coder.typeof(fst),  'sep_fusion_state_t');
    estT  = coder.cstructname(coder.typeof(est),  'sep_estimation_state_t');
    odomT = coder.cstructname(coder.typeof(odom), 'sep_odom_t');

    cfg = coder.config('lib', 'ecoder', false);
    cfg.TargetLang                    = 'C';
    % C89/C90: MATLAB Coder solo emite C99 si el compilador instalado lo
    % soporta, y en Windows el LCC incluido no. C89 es un subconjunto de C99,
    % asi que el codigo compila igual con -std=c99 en la aplicacion
    % (PE-RNF-007); fijarlo aqui evita que el resultado dependa de la PC.
    cfg.TargetLangStandard            = 'C89/C90 (ANSI)';
    cfg.GenCodeOnly                   = true;
    cfg.GenerateReport                = true;
    cfg.EnableDynamicMemoryAllocation = false;
    cfg.SaturateOnIntegerOverflow     = true;
    cfg.HardwareImplementation.ProdHWDeviceType   = 'ARM Compatible->ARM 64-bit (LP64)';
    cfg.HardwareImplementation.TargetHWDeviceType = 'ARM Compatible->ARM 64-bit (LP64)';

    fprintf('Generando el nucleo C en %s\n', outdir);
    codegen('-config', cfg, '-d', outdir, ...
            'sep_fusion_init', ...
            'sep_fusion_step', '-args', {fstT, 0, 0, 0, 0, zeros(1,3), true, geoT, agT}, ...
            'sep_estimation_init', '-args', {prmT}, ...
            'sep_estimation_step', '-args', {estT, odomT, prmT, agT}, ...
            '-report');
    fprintf('Listo. Informe: %s\n', fullfile(outdir, 'html', 'report.mldatx'));
end

% ---------------------------------------------------------------------
function g = local_geo(geo)
%LOCAL_GEO  Campos de sep_geo_params que lee sep_odometry.
    g.m_per_tick_R = geo.m_per_tick_R;
    g.m_per_tick_L = geo.m_per_tick_L;
    g.B_eff        = geo.B_eff;
    g.stop_ticks   = geo.stop_ticks;
end

% ---------------------------------------------------------------------
function a = local_ag(ag)
%LOCAL_AG  Campos de los agentes que leen sep_fusion_step y sep_estimation_step.
    a.gyro_scale    = ag.gyro_scale;
    a.dt_reject     = ag.dt_reject;
    a.dcount_reject = ag.dcount_reject;
    a.loop_ms       = ag.loop_ms;
    a.reset_window  = ag.reset_window;
    a.k_reref       = ag.k_reref;
    a.n_still       = ag.n_still;
    a.tilt_rollover = ag.tilt_rollover;
    a.t_gap_max     = ag.t_gap_max;
end
