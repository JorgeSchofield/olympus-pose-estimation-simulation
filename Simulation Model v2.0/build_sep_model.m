function build_sep_model(modelName, Ts)
%BUILD_SEP_MODEL  Crea programaticamente el modelo Simulink del SEP.
%
%   build_sep_model()                      % 'sep_ekf_rover', Ts = 0.020
%
%   QUE ENTRA AL MODELO Y QUE NO
%   ----------------------------
%   Simulink modela el AGENTE DE ESTIMACION, que es la parte que se genera
%   como C para el HLC. El LLC, el canal y los otros tres agentes se
%   ejecutan en MATLAB (run_sep_demo) y alimentan el modelo por From
%   Workspace. La razon es que esas tres piezas son de tiempo IRREGULAR --
%   el LLC no tiene reloj, el canal pierde tramas, los agentes tienen
%   jitter -- y forzarlas a un solver de paso fijo destruiria justamente lo
%   que se quiere estudiar.
%
%   Por eso dt entra como SENAL y no como Constant: es el dt que reporto el
%   LLC en cada trama, que no es Ts ni es constante. Un modelo con dt fijo
%   no puede reproducir el efecto del reloj de software del firmware.
%
%   Estructura generada:
%
%     [From Workspace: ds_log  ]--->(1)                   /--> [To Ws: x_est_log]
%     [From Workspace: dth_log ]--->(2)  [ sepEKF ] -x_est<
%     [From Workspace: gyro_log]--->(3)      |            \--> [Demux 5]->[Scope]
%     [From Workspace: dt_log  ]--->(4)      \--diag----------> [To Ws: diag_log]
%     [From Workspace: mov_log ]--->(5)
%
%   Variables que leen los bloques (workspace base), generadas por
%   run_sep_demo a partir de la salida de hlc_agents:
%     ds_log   : [t, ds]      desplazamiento por paso [m]
%     dth_log  : [t, dth]     giro por odometria [rad]
%     gyro_log : [t, w_gyro]  tasa de yaw [rad/s]
%     dt_log   : [t, dt]      intervalo REPORTADO por el LLC [s]
%     mov_log  : [t, moving]  1 si hay movimiento

    if nargin < 1 || isempty(modelName), modelName = 'sep_ekf_rover'; end
    if nargin < 2 || isempty(Ts),        Ts = 0.020;                  end

    if bdIsLoaded(modelName), close_system(modelName, 0); end
    if exist([modelName '.slx'], 'file'), delete([modelName '.slx']); end

    new_system(modelName);
    set_param(modelName, ...
        'SolverType', 'Fixed-step', ...
        'Solver',     'FixedStepDiscrete', ...
        'FixedStep',  num2str(Ts), ...
        'StartTime',  '0', ...
        'StopTime',   '100');

    pth = [modelName '/'];

    src = {'ds_log','Odometria_ds',  'Setting to zero'
           'dth_log','Odometria_dth','Setting to zero'
           'gyro_log','Giroscopio',  'Holding final value'
           'dt_log','dt_reportado',  'Holding final value'
           'mov_log','Movimiento',   'Holding final value'};
    for i = 1:size(src,1)
        add_block('simulink/Sources/From Workspace', [pth src{i,2}], ...
            'Position', [40 40+70*(i-1) 175 85+70*(i-1)], ...
            'VariableName', src{i,1}, 'SampleTime', num2str(Ts), ...
            'Interpolate', 'off', 'OutputAfterFinalValue', src{i,3});
    end

    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [pth 'sepEKF'], 'Position', [320 120 520 280]);
    setMatlabFunctionCode(modelName, 'sepEKF', blockCodeLines());

    add_block('simulink/Sinks/To Workspace', [pth 'x_est'], ...
        'Position', [620 300 720 340], 'VariableName', 'x_est_log', ...
        'SaveFormat', 'Timeseries', 'SampleTime', num2str(Ts));
    add_block('simulink/Sinks/To Workspace', [pth 'diag'], ...
        'Position', [620 360 720 400], 'VariableName', 'diag_log', ...
        'SaveFormat', 'Timeseries', 'SampleTime', num2str(Ts));

    % El estado se separa por unidades: metros, radianes y rad/s no
    % comparten eje. En un solo trazo el sesgo del giroscopio (0.012) es
    % invisible al lado de la posicion, y es justo la traza que muestra la
    % convergencia del ZARU.
    add_block('simulink/Signal Routing/Demux', [pth 'Demux'], ...
        'Position', [570 130 575 250], 'Outputs', '5');
    add_block('simulink/Signal Routing/Mux', [pth 'Mux_pos'], ...
        'Position', [620 130 625 180], 'Inputs', '2');
    add_block('simulink/Sinks/Scope', [pth 'Scope'], ...
        'Position', [700 120 750 260], 'NumInputPorts', '4');

    for i = 1:size(src,1)
        add_line(modelName, [src{i,2} '/1'], sprintf('sepEKF/%d', i), ...
                 'autorouting', 'on');
    end
    add_line(modelName, 'sepEKF/1', 'x_est/1', 'autorouting', 'on');
    add_line(modelName, 'sepEKF/2', 'diag/1',  'autorouting', 'on');
    add_line(modelName, 'sepEKF/1', 'Demux/1', 'autorouting', 'on');

    nm  = {'px [m]','py [m]','theta [rad]','omega [rad/s]','b_gyro [rad/s]'};
    dst = {'Mux_pos/1','Mux_pos/2','Scope/2','Scope/3','Scope/4'};
    for k = 1:5
        h = add_line(modelName, sprintf('Demux/%d', k), dst{k}, ...
                     'autorouting', 'on');
        set_param(h, 'Name', nm{k});
    end
    add_line(modelName, 'Mux_pos/1', 'Scope/1', 'autorouting', 'on');

    nl = numel(find_system(modelName, 'FindAll','on', 'Type','line'));
    if nl < 13
        warning('build_sep_model:cableadoIncompleto', ...
                'Solo se crearon %d lineas de las 13 esperadas.', nl);
    end

    try
        sc = get_param([pth 'Scope'], 'ScopeConfiguration');
        sc.LayoutDimensions      = [4 1];
        sc.OpenAtSimulationStart = false;
        ejes = {'Posicion [m]','Rumbo [rad]', ...
                'Velocidad angular [rad/s]','Sesgo del giroscopio [rad/s]'};
        for k = 1:4
            sc.ActiveDisplay = k;
            sc.YLabel = ejes{k};  sc.Title = ejes{k};  sc.ShowLegend = true;
        end
    catch
        warning(['No se pudo configurar el Scope en cuatro ejes ' ...
                 '(diferencia de version). El modelo funciona igual.']);
    end

    Simulink.BlockDiagram.arrangeSystem(modelName);
    save_system(modelName);
    fprintf('Modelo Simulink generado: %s.slx\n', modelName);
end

% ---------------------------------------------------------------------
function setMatlabFunctionCode(modelName, blockName, lines)
    code  = strjoin(lines, newline);
    rt    = sfroot;
    chart = rt.find('-isa','Stateflow.EMChart','Path',[modelName '/' blockName]);
    chart.Script = code;
end

% ---------------------------------------------------------------------
function lines = blockCodeLines()
% Codigo del bloque sepEKF. Sin comillas simples internas para evitar
% problemas de escape al inyectarlo.
%
% Restricciones de codegen que hay que respetar aqui:
%  1) Solo x y P son persistentes. prm NO: si se declara persistente y se
%     asigna dentro de if isempty(x), el generador no puede probar que este
%     definida en todas las rutas y rechaza el bloque. Es constante, asi
%     que la pliega igual.
%  2) La declaracion persistent debe ser la PRIMERA sentencia. El parser
%     del bloque MATLAB Function es mas estricto que el de un archivo .m.
%  3) Las salidas se predimensionan y se rellenan con (:) para que Simulink
%     infiera tamano y tipo sin analizar el cuerpo entero.
    lines = {
        'function [x_est, diag_out] = sepEKF(ds, dth, w_gyro, dt, moving)'
        '%#codegen'
        'persistent x P'
        'prm = sep_ekf_params();'
        'if isempty(x)'
        '    x = zeros(5,1);'
        '    P = diag(prm.P0);'
        'end'
        'x_est = zeros(5,1);'
        'diag_out = zeros(3,1);'
        '[x, P, xe, dg] = sep_ekf_step(x, P, ds, dth, w_gyro, dt, moving, prm);'
        'x_est(:) = xe;'
        'diag_out(:) = dg;'
        'end'
    };
end
