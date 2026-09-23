function build_sep_full_model(modelName, Tb)
%BUILD_SEP_FULL_MODEL  Modelo Simulink COMPLETO del rover: LLC + enlace + HLC.
%
%   build_sep_full_model()                 % 'sep_rover_full', Tb = 0.0005
%
%   Contiene los dos controladores y el enlace entre ellos. Es el artefacto
%   demostrable del trabajo: se abre, se ve la frontera LLC/HLC, se conmuta
%   clock_mode de software a timer y el error de estimacion cambia delante
%   de quien lo este mirando.
%
%   =====================================================================
%   POR QUE PASO BASE FINO Y NO EL PERIODO DEL LLC
%   =====================================================================
%   El ciclo del LLC NO dura lo que dice durar: delay_ms(LOOP_MS) al final
%   del lazo es un hueco, no un periodo, asi que el ciclo dura LOOP_MS mas
%   todo el trabajo. Un solver de paso fijo al periodo del LLC no podria
%   representar eso.
%
%   Con Tb = 0.5 ms el problema desaparece: un ciclo de duracion variable
%   es simplemente un numero variable de pasos base. Cada subsistema corre
%   a Tb y decide internamente si le toca actuar.
%
%   COSTE Y LIMITE
%     600 000 pasos para 300 s de simulacion. Usar modo Accelerator.
%     Los ciclos quedan cuantizados a multiplos de Tb: con ciclos de ~30 ms
%     eso es un 1.6 %, muy por debajo del ~50 % de error de reloj que es el
%     objeto de estudio. Pero significa que el modelo NO resuelve efectos
%     mas finos que Tb: el tiempo de un byte a 115200 son 86.8 us, asi que
%     la transmision se modela a nivel de trama, no de byte.
%
%   =====================================================================
%   EL RUIDO ENTRA POR FROM WORKSPACE, NO POR BLOQUES RANDOM NUMBER
%   =====================================================================
%   Si Simulink generase su propio ruido se perderia la verificacion
%   cruzada contra el bucle de MATLAB. Con las secuencias inyectadas como
%   senal, las dos implementaciones dan resultados IDENTICOS y
%   max|Simulink - MATLAB| vuelve a ser una metrica con sentido. Dos
%   implementaciones independientes que coinciden son evidencia; una sola
%   es solo codigo.
%
%   =====================================================================
%   FRONTERA DE CODEGEN
%   =====================================================================
%   Solo cuatro archivos tienen que ser codegen-safe, porque son los que se
%   generan como C para el HLC: sep_geo_params, sep_ekf_params,
%   sep_odometry y sep_ekf_step. El bloque LLC es un EMULADOR del firmware
%   de otra persona y no se genera nunca: no hay razon para mantenerlo
%   codegen-clean, y conviene decirlo aqui para que nadie lo intente.
%
%   ESTRUCTURA GENERADA
%
%     [From Ws: cmd_log  ]--->(1) ┌─────────┐
%     [From Ws: terr_log ]--->(2) │ Planta  │--- pose, arco, w, a, tilt
%                                 └─────────┘         │
%     [From Ws: noise_log]--->(2) ┌─────────┐<--------┘
%                                 │   LLC   │--- buf(uint8), n, emit
%                                 └─────────┘         │
%     [From Ws: chan_log ]--->(4) ┌─────────┐<--------┘
%                                 │  Canal  │--- buf, n, arrived
%                                 └─────────┘         │
%                                 ┌─────────┐<--------┘
%                                 │   HLC   │--- pose, P, diag, lat, pub
%                                 └─────────┘
%              [To Workspace: pose_log, lat_log, diag_log, truth_log]
%
%   Variables del workspace base, generadas por run_sep_full_demo:
%     cmd_log   : [t, v_cmd, w_cmd]
%     terr_log  : [t, pitch, roll, slip1..slip6]
%     noise_log : [t, n1..n5]          ruido de la IMU
%     chan_log  : [t, u1, u2, u3]      uniformes del canal

    if nargin < 1 || isempty(modelName), modelName = 'sep_rover_full'; end
    if nargin < 2 || isempty(Tb),        Tb = 0.0005;                  end

    if bdIsLoaded(modelName), close_system(modelName, 0); end
    if exist([modelName '.slx'], 'file'), delete([modelName '.slx']); end

    new_system(modelName);
    set_param(modelName, ...
        'SolverType','Fixed-step', 'Solver','FixedStepDiscrete', ...
        'FixedStep', num2str(Tb), 'StartTime','0', 'StopTime','300', ...
        'SimulationMode','accelerator');

    p = [modelName '/'];
    Ts = num2str(Tb);

    % ================= FUENTES =================
    src = {'cmd_log',  'Comando',   [40  40],  'Holding final value'
           'terr_log', 'Terreno',   [40 130],  'Holding final value'
           'noise_log','Ruido_IMU', [40 220],  'Holding final value'
           'chan_log', 'Azar_canal',[40 310],  'Holding final value'};
    for i = 1:size(src,1)
        pos = src{i,3};
        add_block('simulink/Sources/From Workspace', [p src{i,2}], ...
            'Position', [pos(1) pos(2) pos(1)+140 pos(2)+45], ...
            'VariableName', src{i,1}, 'SampleTime', Ts, ...
            'Interpolate','off', 'OutputAfterFinalValue', src{i,4});
    end

    % ================= SUBSISTEMAS =================
    blk = {'Planta',  [250  40 420 170], @plantaCode
           'LLC',     [250 200 420 330], @llcCode
           'Canal',   [250 360 420 470], @canalCode
           'HLC',     [250 500 420 640], @hlcCode};
    for i = 1:size(blk,1)
        add_block('simulink/User-Defined Functions/MATLAB Function', ...
                  [p blk{i,1}], 'Position', blk{i,2});
        setCode(modelName, blk{i,1}, blk{i,3}());
    end

    % ================= SUMIDEROS =================
    snk = {'pose_log','Pose',   [520 500]
           'lat_log', 'Latencia',[520 560]
           'diag_log','Diag',   [520 620]
           'truth_log','Verdad',[520  60]};
    for i = 1:size(snk,1)
        pos = snk{i,3};
        add_block('simulink/Sinks/To Workspace', [p snk{i,2}], ...
            'Position', [pos(1) pos(2) pos(1)+100 pos(2)+40], ...
            'VariableName', snk{i,1}, 'SaveFormat','Timeseries', ...
            'SampleTime', Ts);
    end

    % ================= CABLEADO =================
    L = { 'Comando/1','Planta/1' ; 'Terreno/1','Planta/2'
          'Planta/1','LLC/1'     ; 'Planta/2','LLC/2'
          'Planta/3','LLC/3'     ; 'Ruido_IMU/1','LLC/4'
          'LLC/1','Canal/1'      ; 'LLC/2','Canal/2'
          'LLC/3','Canal/3'      ; 'Azar_canal/1','Canal/4'
          'Canal/1','HLC/1'      ; 'Canal/2','HLC/2'
          'Canal/3','HLC/3'      ; 'Azar_canal/1','HLC/4'
          'HLC/1','Pose/1'       ; 'HLC/2','Latencia/1'
          'HLC/3','Diag/1'       ; 'Planta/1','Verdad/1' };
    for i = 1:size(L,1)
        add_line(modelName, L{i,1}, L{i,2}, 'autorouting','on');
    end

    nl = numel(find_system(modelName,'FindAll','on','Type','line'));
    if nl < size(L,1)
        warning('build_sep_full_model:cableadoIncompleto', ...
                'Solo se crearon %d lineas de las %d esperadas.', nl, size(L,1));
    end

    Simulink.BlockDiagram.arrangeSystem(modelName);
    save_system(modelName);
    fprintf(['Modelo completo generado: %s.slx\n' ...
             '  paso base %.1f ms, modo Accelerator.\n' ...
             '  Ejecutar con run_sep_full_demo.\n'], modelName, 1000*Tb);
end

% ---------------------------------------------------------------------
function setCode(modelName, blockName, lines)
    rt    = sfroot;
    chart = rt.find('-isa','Stateflow.EMChart','Path',[modelName '/' blockName]);
    chart.Script = strjoin(lines, newline);
end

% ---------------------------------------------------------------------
% Los bloques son ENVOLTURAS DELGADAS: mantienen el estado persistente y
% delegan en las funciones de paso base, que son la fuente unica y las
% mismas que usa el bucle de MATLAB. Nada de logica duplicada aqui.
%
% Restricciones de codegen que hay que respetar en un bloque MATLAB
% Function: la declaracion persistent debe ser la PRIMERA sentencia; las
% salidas se predimensionan para que Simulink infiera tamano y tipo sin
% analizar el cuerpo entero; y no se anaden campos a un struct despues de
% haberlo leido.
%
% Los bloques llaman a sim_params, llc_params, sep_geo_params y
% sep_ekf_params DIRECTAMENTE, nunca a rover_params: esa usa fprintf y
% warning para la verificacion de consistencia, y rover_params_quiet la
% silencia con evalc. Ninguna de las dos cosa puede vivir dentro de un
% bloque MATLAB Function.
% ---------------------------------------------------------------------
function c = plantaCode()
    c = {
    'function [pose, arcw, kin] = Planta(cmd, terr)'
    'persistent st sp'
    'if isempty(st)'
    '    sp = sim_params();'
    '    st = plant_init();'
    'end'
    'pose = zeros(1,3); arcw = zeros(1,6); kin = zeros(1,5);'
    '[st, o] = plant_step(st, cmd(1), cmd(2), terr(3:8), terr(1), terr(2), ...'
    '                     sp.sim.Tb, sp.plant);'
    'pose(:) = o.pose;'
    'arcw(:) = o.arc;'
    'kin(:)  = [o.w, o.a_body, terr(1), terr(2)];'
    'end'
    };
end

function c = llcCode()
    c = {
    'function [buf, n, emit] = LLC(pose, arcw, kin, noise)'
    'persistent st sp lp gp'
    'if isempty(st)'
    '    sp = sim_params(); lp = llc_params(); gp = sep_geo_params();'
    '    st = llc_init();'
    'end'
    'buf = zeros(1, raw_buf_max(), ''uint8''); n = 0; emit = false;'
    '[st, b, nn, e] = llc_step(st, kin(1), kin(2:3), kin(4), kin(5), arcw, ...'
    '                          noise, lp, gp, sp.imu, sp.sim.Tb);'
    'buf(:) = b; n = nn; emit = e;'
    'end'
    };
end

function c = canalCode()
    c = {
    'function [obuf, on, arrived] = Canal(buf, n, emit, u)'
    'persistent st sp'
    'if isempty(st)'
    '    sp = sim_params();'
    '    st = channel_init(sp.chan);'
    'end'
    'obuf = zeros(1, raw_buf_max(), ''uint8''); on = 0; arrived = false;'
    '[st, b, nn, a] = channel_step(st, buf, n, emit, u, sp.chan, sp.sim.Tb);'
    'obuf(:) = b; on = nn; arrived = a;'
    'end'
    };
end

function c = hlcCode()
    c = {
    'function [pose, lat, diag_out] = HLC(buf, n, arrived, u)'
    'persistent st sp gp ep'
    'if isempty(st)'
    '    sp = sim_params(); gp = sep_geo_params(); ep = sep_ekf_params();'
    '    st = hlc_init(ep);'
    'end'
    'pose = zeros(5,1); lat = 0; diag_out = zeros(3,1);'
    '[st, x, ~, dg, l, ~] = hlc_step(st, buf, n, arrived, ...'
    '                                gp, ep, sp.agents, sp.sim.Tb, u(3));'
    'pose(:) = x; diag_out(:) = dg; lat = l;'
    'end'
    };
end
