function build_sep_model(modelName, Ts)
%BUILD_SEP_MODEL  Crea programaticamente el modelo Simulink del SEP.
%
%   build_sep_model()                      % 'sep_ekf_rover', Ts = 0.020
%   build_sep_model('mi_modelo', 0.02)
%
%   Estructura generada:
%
%     [From Workspace: enc_log ]--->(1)                       /--> [To Workspace: x_est_log]
%     [From Workspace: gyro_log]--->(2)  [ sepEKF ] --x_est--<
%     [Constant      : dt (=Ts)]--->(3)      |                \--> [Demux 5] --> [Scope, 3 ejes]
%                                            \--diag--------------> [To Workspace: diag_log]
%
%   El Scope separa el estado en tres ejes porque las cinco variables no
%   comparten unidades: posicion [m], rumbo [rad] y velocidades [rad/s].
%
%   El bloque sepEKF mantiene (x,P) como PERSISTENTES y delega en
%   sep_odometry, sep_ekf_step, sep_geo_params y sep_ekf_params, que deben
%   estar en el path.
%
%   El GPS NO aparece en el modelo: es verdad de terreno offline
%   (DRT-SEP-001, PE-CON-001). run_sep_demo.m lo genera y lo usa solo en
%   las metricas y las graficas.
%
%   Variables que leen los bloques From Workspace (workspace base):
%     enc_log  : [t, d_FR, d_FL, d_CR, d_CL, d_RR, d_RL]   (Nx7)
%     gyro_log : [t, wz]                                    (Nx2)

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

    p = [modelName '/'];

    % ================= FUENTES =================
    add_block('simulink/Sources/From Workspace', [p 'Encoders'], ...
        'Position', [40  40 170  90], 'VariableName', 'enc_log', ...
        'SampleTime', num2str(Ts), 'Interpolate', 'off', ...
        'OutputAfterFinalValue', 'Setting to zero');

    add_block('simulink/Sources/From Workspace', [p 'Gyro'], ...
        'Position', [40 120 170 170], 'VariableName', 'gyro_log', ...
        'SampleTime', num2str(Ts), 'Interpolate', 'off', ...
        'OutputAfterFinalValue', 'Holding final value');

    add_block('simulink/Sources/Constant', [p 'dt'], ...
        'Position', [40 200 170 240], 'Value', num2str(Ts), ...
        'SampleTime', num2str(Ts));

    % ================= BLOQUE EKF =================
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [p 'sepEKF'], 'Position', [300 90 500 220]);
    setMatlabFunctionCode(modelName, 'sepEKF', blockCodeLines());

    % ================= SUMIDEROS =================
    add_block('simulink/Sinks/To Workspace', [p 'x_est'], ...
        'Position', [560 260 660 300], 'VariableName', 'x_est_log', ...
        'SaveFormat', 'Timeseries', 'SampleTime', num2str(Ts));

    add_block('simulink/Sinks/To Workspace', [p 'diag'], ...
        'Position', [560 320 660 360], 'VariableName', 'diag_log', ...
        'SaveFormat', 'Timeseries', 'SampleTime', num2str(Ts));

    % Separar el estado por unidades: metros, radianes y rad/s no comparten
    % eje. En un solo trazo el sesgo del giroscopio (0.012) es invisible al
    % lado de la posicion (0 a 2 m).
    add_block('simulink/Signal Routing/Demux', [p 'Demux'], ...
        'Position', [520 100 525 200], 'Outputs', '5');

    add_block('simulink/Signal Routing/Mux', [p 'Mux_pos'], ...
        'Position', [570 100 575 130], 'Inputs', '2');

    add_block('simulink/Sinks/Scope', [p 'Scope'], ...
        'Position', [630 130 670 170]);

    % El Scope nace con UN puerto. Hay que ampliarlo a cuatro ANTES de cablear:
    % conectar a Scope/2 cuando ese puerto todavia no existe hace fallar
    % add_line y deja el modelo a medio construir, con bloques y sin lineas.
    try
        set_param([p 'Scope'], 'NumInputPorts', '4');
    catch
        sc0 = get_param([p 'Scope'], 'ScopeConfiguration');
        sc0.NumInputPorts = '4';
    end

    % ================= CONEXIONES =================
    add_line(modelName, 'Encoders/1', 'sepEKF/1', 'autorouting', 'on');
    add_line(modelName, 'Gyro/1',     'sepEKF/2', 'autorouting', 'on');
    add_line(modelName, 'dt/1',       'sepEKF/3', 'autorouting', 'on');
    add_line(modelName, 'sepEKF/1',   'x_est/1',  'autorouting', 'on');
    add_line(modelName, 'sepEKF/2',   'diag/1',   'autorouting', 'on');
    add_line(modelName, 'sepEKF/1',   'Demux/1',  'autorouting', 'on');

    % Nombrar cada linea: el nombre de la senal es lo que aparece en la
    % leyenda del Scope. Sin esto la leyenda dice "Signal 1", "Signal 2".
    nm = {'px [m]', 'py [m]', 'theta [rad]', 'omega [rad/s]', 'b_gyro [rad/s]'};
    % El sesgo del giroscopio va en su PROPIO eje: 0.012 rad/s junto a una
    % velocidad angular de 0.35 rad/s es una razon de 30 a 1 y se confunde
    % con el cero, justo la traza que muestra la convergencia del ZARU.
    dst = {'Mux_pos/1', 'Mux_pos/2', 'Scope/2', 'Scope/3', 'Scope/4'};
    for k = 1:5
        h = add_line(modelName, sprintf('Demux/%d', k), dst{k}, ...
                     'autorouting', 'on');
        set_param(h, 'Name', nm{k});
    end
    add_line(modelName, 'Mux_pos/1', 'Scope/1', 'autorouting', 'on');

    % Comprobacion: si algo quedo sin conectar, avisar en vez de guardar un
    % modelo roto en silencio.
    nl = numel(find_system(modelName, 'FindAll', 'on', 'Type', 'line'));
    if nl < 9
        warning('build_sep_model:cableadoIncompleto', ...
                'Solo se crearon %d lineas de las 9 esperadas.', nl);
    end

    % Configuracion cosmetica del Scope. Va en try/catch porque los nombres
    % de propiedad de ScopeConfiguration han cambiado entre versiones y una
    % diferencia de version no debe impedir que el modelo se construya.
    try
        sc = get_param([p 'Scope'], 'ScopeConfiguration');
        sc.LayoutDimensions      = [4 1];
        sc.OpenAtSimulationStart = false;
        ejes = {'Posicion [m]', 'Rumbo [rad]', ...
                'Velocidad angular [rad/s]', 'Sesgo del giroscopio [rad/s]'};
        for k = 1:4
            sc.ActiveDisplay = k;
            sc.YLabel        = ejes{k};
            sc.Title         = ejes{k};
            sc.ShowLegend    = true;   % es por display, no global
        end
    catch
        warning(['No se pudo configurar el Scope en cuatro ejes ' ...
                 '(diferencia de version). El modelo funciona igual; ' ...
                 'ajustar Layout y Legend a mano desde el Scope.']);
    end

    Simulink.BlockDiagram.arrangeSystem(modelName);
    save_system(modelName);
    fprintf('Modelo Simulink generado y guardado: %s.slx\n', modelName);
end

% ---------------------------------------------------------------------
function setMatlabFunctionCode(modelName, blockName, lines)
% Inyecta el codigo (cell de lineas) en un bloque MATLAB Function.
    code  = strjoin(lines, newline);
    rt    = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', ...
                    'Path', [modelName '/' blockName]);
    chart.Script = code;
end

% ---------------------------------------------------------------------
function lines = blockCodeLines()
% Codigo del bloque sepEKF. Sin comillas simples internas para evitar
% problemas de escape al inyectarlo.
%
% Dos restricciones de codegen que hay que respetar aqui:
%  1) Solo x y P son persistentes. geo y prm NO lo son: si se declaran
%     persistentes y se asignan dentro de if isempty(x), el generador no
%     puede probar que esten definidas en todas las rutas y rechaza el
%     bloque. Son constantes, asi que el generador las pliega igual.
%  2) La declaracion persistent debe ser la PRIMERA sentencia de la
%     funcion. El parser del bloque MATLAB Function es mas estricto que
%     el de un archivo .m normal y rechaza declaraciones colocadas
%     despues de sentencias ejecutables.
%  3) Las salidas se predimensionan y se rellenan con (:) para que
%     Simulink pueda inferir tamano y tipo sin analizar el cuerpo entero.
    lines = {
        'function [x_est, diag_out] = sepEKF(enc_delta, w_gyro, dt)'
        '%#codegen'
        'persistent x P'
        'geo = sep_geo_params();'
        'prm = sep_ekf_params();'
        'if isempty(x)'
        '    x = zeros(5,1);'
        '    P = diag(prm.P0);'
        'end'
        'x_est = zeros(5,1);'
        'diag_out = zeros(3,1);'
        '[ds, dth, moving, ~] = sep_odometry(enc_delta(:), geo);'
        '[x, P, xe, dg] = sep_ekf_step(x, P, ds, dth, w_gyro, dt, moving, prm);'
        'x_est(:) = xe;'
        'diag_out(:) = dg;'
        'end'
    };
end