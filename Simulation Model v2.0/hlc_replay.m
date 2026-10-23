function R = hlc_replay(fname, varargin)
%HLC_REPLAY  Procesa una grabacion del enlace igual que `olympus-pose -r`.
%
%   R = HLC_REPLAY(fname)
%   R = HLC_REPLAY(fname, 'dt_source', 1, 'gyro_yaw_axis', 2, 'csv', 'out.csv')
%
%   fname : grabacion de llcmux (llc_record.txt), salida de
%           capture_to_replay.py o de llc_emulator.py --out. Lineas
%           "[@<t_rx_ns> ]RAW:..."; las demas (TLM, ACK, ...) se ignoran.
%
%   Recorre la MISMA cadena que la aplicacion en modo replay, donde ningun
%   mensaje se descarta y la estimacion es por evento:
%
%       raw_frame_from_bytes  (= pe_raw_parse)
%       hlc_fusion_inputs     (= pe_fusion_inputs)
%       sep_fusion_step       (mismo codigo que genera MATLAB Coder)
%       sep_estimation_step   (idem), una vez por mensaje de fusion
%
%   Una fila por pose publicada (did = 1), como pose.csv de la aplicacion:
%       line t_rx_ns tick t_in S x y theta omega bias d1 d2 d3 still imu_ok
%   Comparar con pose.csv: x_m, y_m, theta_rad, ... deben coincidir con la
%   precision de impresion. agents/tests/test_core.c hace la misma
%   comparacion con el C (export_replay_vectors.m genera los vectores).
%
%   Los parametros vienen del modelo (sep_geo_params, sep_ekf_params,
%   sim_params), los mismos que export_pose_conf vuelca en pose.conf.
%   'dt_source' y 'gyro_yaw_axis' permiten repetir una corrida con otra
%   base de tiempo o montaje de la IMU sin editar sim_params.

    op = struct('dt_source', [], 'gyro_yaw_axis', [], 'csv', '');
    for i = 1:2:numel(varargin)
        op.(varargin{i}) = varargin{i+1};
    end
    geo = sep_geo_params();
    prm = sep_ekf_params();
    sp  = sim_params();  ag = sp.agents;
    if ~isempty(op.dt_source),     ag.dt_source     = op.dt_source;     end
    if ~isempty(op.gyro_yaw_axis), ag.gyro_yaw_axis = op.gyro_yaw_axis; end

    L   = readlines(fname);
    fst = sep_fusion_init();
    est = sep_estimation_init(prm);
    R   = zeros(numel(L), 15);
    nr  = 0;
    nbad = 0;
    for i = 1:numel(L)
        s = char(L(i));
        t_rx_ns = NaN;
        if ~isempty(s) && s(1) == '@'
            sp_ = find(s == ' ', 1);
            tok = s(2:max(1, sp_-1));
            if isempty(sp_) || sp_ < 3 || any(tok < '0' | tok > '9')
                nbad = nbad + 1;  continue;        % prefijo invalido: BAD
            end
            t_rx_ns = sscanf(tok, '%lu');
            s = s(sp_+1:end);
        end
        b = uint8(s);
        [f, ok] = raw_frame_from_bytes(b, numel(b));
        if ~ok
            if numel(s) >= 4 && strcmp(s(1:4), 'RAW:'), nbad = nbad + 1; end
            continue;
        end
        if isnan(t_rx_ns)
            % sin prefijo la aplicacion usaria su reloj local; en replay no
            % hay reloj de llegada, asi que solo tiene sentido dt_source = 0
            assert(ag.dt_source == 0, 'hlc_replay: %s no trae "@t" y dt_source = 1', fname);
            t_rx_ms = 0;
        else
            t_rx_ms = floor(double(t_rx_ns) / 1e6);
        end
        [t_in, gz, acc, imu_ok] = hlc_fusion_inputs(f, t_rx_ms, ag);
        [fst, odom, emit] = sep_fusion_step(fst, t_in, f.encL, f.encR, gz, acc, imu_ok, geo, ag);
        if ~emit, continue; end
        [est, xe, dg, did] = sep_estimation_step(est, odom, prm, ag);
        if ~did, continue; end
        nr = nr + 1;
        R(nr,:) = [i, double(t_rx_ns), f.tick_ms, t_in, odom.S, xe(:).', dg(:).', ...
                   double(odom.still), double(imu_ok)];
    end
    R = R(1:nr,:);
    fprintf('hlc_replay: %d lineas, %d poses, %d tramas RAW invalidas, %d rechazos de fusion\n', ...
            numel(L), nr, nbad, fst.n_reject);
    if ~isempty(op.csv)
        fid = fopen(op.csv, 'w');
        assert(fid > 0, 'no se pudo abrir %s', op.csv);
        fprintf(fid, 'line,t_rx_ns,tick,t_in,S,x,y,theta,omega,bias,d1,d2,d3,still,imu_ok\n');
        fprintf(fid, ['%d,%.0f,%.0f,%.0f' repmat(',%.17g', 1, 9) ',%d,%d\n'], R.');
        fclose(fid);
    end
end
