function test_params_contract()
%TEST_PARAMS_CONTRACT  Los parametros que el codigo LEE deben EXISTIR.
%
%   Ejecutar:  >> test_params_contract
%
%   POR QUE EXISTE ESTA PRUEBA
%   --------------------------
%   MATLAB resuelve los campos de struct en tiempo de ejecucion, asi que un
%   campo ausente no falla al cargar: falla en medio de una corrida larga,
%   con un "Unrecognized field name" y la pila a medio camino.
%
%   Peor aun, falla de forma ASIMETRICA. Los bloques de Simulink llaman a
%   sim_params() directamente y los scripts llaman a rover_params(): si un
%   campo se anade solo a uno de los dos, una ruta funciona y la otra no, y
%   la causa no se parece al sintoma.
%
%   Esta prueba recorre los campos que cada funcion de paso lee y comprueba
%   que las dos rutas los proveen. Es barata y atrapa toda esa clase.
    n = 0;

    sp  = sim_params();
    geo = sep_geo_params();
    ekf = sep_ekf_params();
    llc = llc_params();

    req = { 'chan',   {'byte_s','p_loss','p_corrupt','poll_s','max_inflight'}
            'agents', {'gyro_scale','age_est','est_period','est_delay', ...
                       'com_delay','jitter','dt_reject','dcount_reject'}
            'plant',  {'half_track','chi_true','terrain','slip'}
            'sim',    {'Tb'} };

    for i = 1:size(req,1)
        grp = req{i,1};
        assert(isfield(sp, grp), 'sim_params no define el grupo .%s', grp);
        for f = req{i,2}
            assert(isfield(sp.(grp), f{1}), ...
                'sim_params().%s NO define el campo "%s"', grp, f{1});
        end
    end
    n = n + 1;

    for f = {'idx_R','idx_L','m_per_tick_R','m_per_tick_L','B_eff', ...
             'R_wheel','ticks_per_rev','sign_R','sign_L','stop_ticks'}
        assert(isfield(geo, f{1}), 'sep_geo_params no define "%s"', f{1});
    end
    n = n + 1;

    for f = {'P0','dt_min','dt_max','k_rho','s2_ds_floor','s2_theta', ...
             's2_w_enc','s2_bg','r_gyro','r_zaru','slip_thresh', ...
             'slip_gain','slip_cap'}
        assert(isfield(ekf, f{1}), 'sep_ekf_params no define "%s"', f{1});
    end
    n = n + 1;

    for f = {'LOOP_MS','HC_PERIOD','SEN_PERIOD','TLM_PERIOD','STALL_THRESH', ...
             'clock_mode','tx_mode','emit_raw','emit_tlm','byte_s', ...
             'raw_bytes','tlm_bytes','c_misc','c_imu','c_hc','c_adc', ...
             'off_imu','off_enc','clock_ppm','gyro_lsb','accel_lsb', ...
             'i16_max','dlpf_bw'}
        assert(isfield(llc, f{1}), 'llc_params no define "%s"', f{1});
    end
    n = n + 1;

    % --- las dos rutas deben coincidir ----------------------------------
    %   rover_params envuelve a sim_params. Si una anade un campo y la otra
    %   no, Simulink y los scripts divergen.
    p = rover_params_quiet();
    for i = 1:size(req,1)
        grp = req{i,1};
        for f = req{i,2}
            assert(isfield(p.(grp), f{1}), ...
                'rover_params().%s pierde el campo "%s" de sim_params', grp, f{1});
        end
    end
    n = n + 1;

    % --- byte_s no debe duplicarse --------------------------------------
    %   Su fuente unica es llc_params, que conoce el baudrate. Si alguien lo
    %   copia a mano, baud y byte_s pueden desincronizarse en silencio.
    assert(abs(sp.chan.byte_s - llc.byte_s) < 1e-12, ...
        'chan.byte_s no coincide con llc_params: hay una copia a mano');
    assert(abs(llc.byte_s - 10/llc.baud) < 1e-12, ...
        'llc.byte_s no corresponde a llc.baud');
    n = n + 1;

    % --- la cola del canal debe honrar el parametro ---------------------
    %   Estuvo fijada a 4 en el codigo mientras el parametro decia 16.
    ch = sp.chan;  ch.max_inflight = 7;
    st = channel_init(ch);
    assert(numel(st.busy) == 7, ...
        'channel_init ignora ch.max_inflight (cola de %d)', numel(st.busy));
    n = n + 1;

    % --- higiene de codegen en las cuatro fuentes -----------------------
    %   Regla: no se puede ANADIR un campo a un struct despues de haberlo
    %   LEIDO, y leer un campo en el lado derecho cuenta como lectura. Un
    %   solo campo que la infrinja invalida todos los que vengan detras, y
    %   el error aparece en el bloque de Simulink que llamo a la fuente,
    %   muy lejos de la causa. Esta comprobacion la atrapa en el archivo.
    for f = {'llc_params.m','sim_params.m','sep_ekf_params.m','sep_geo_params.m'}
        [bad, ln, fld] = local_scan(f{1});
        assert(~bad, ['%s linea %d: anade el campo "%s" despues de leer ' ...
                      'el struct. Calcular en variables LOCALES y rellenar ' ...
                      'el struct al final.'], f{1}, ln, fld);
    end
    n = n + 1;

    % --- nada de strcmpi en el codigo que llega a los bloques -----------
    %   Comparar cadenas obliga a llevar campos char en el struct de
    %   parametros, que codegen trata como arreglos de tamano variable.
    %   llc_params deriva sw_clock y tx_block justamente para evitarlo.
    for f = {'llc_step.m','plant_step.m','channel_step.m','hlc_step.m', ...
             'sep_odometry.m','sep_ekf_step.m'}
        src = fileread(f{1});
        src = regexprep(src, '%[^\n]*', '');        % fuera los comentarios
        assert(isempty(strfind(src, 'strcmpi')) && ...
               isempty(strfind(src, 'strcmp(')), ...
            ['%s compara cadenas. Usar las banderas numericas ' ...
             '(llc.sw_clock, llc.tx_block).'], f{1});
    end
    n = n + 1;

    fprintf('OK: %d comprobaciones de contrato de parametros superadas.\n', n);
end

% ---------------------------------------------------------------------
function [bad, badln, badfld] = local_scan(fname)
%LOCAL_SCAN  Busca "anadir un campo despues de leer el struct".
    bad = false;  badln = 0;  badfld = '';
    txt = fileread(fname);
    lines = strsplit(txt, newline);
    tok = regexp(lines{1}, 'function\s+(\w+)\s*=', 'tokens', 'once');
    if isempty(tok), return; end
    v = tok{1};
    seen = {};  readAt = 0;
    for i = 1:numel(lines)
        code = lines{i};
        c = strfind(code, '%');
        if ~isempty(c), code = code(1:c(1)-1); end
        if isempty(strtrim(code)), continue; end
        lhs = regexp(code, ['^\s*' v '\.(\w+)\s*='], 'tokens', 'once');
        if isempty(lhs)
            rhs = code;
        else
            eq = strfind(code, '=');
            rhs = code(eq(1)+1:end);
        end
        if readAt == 0 && ~isempty(regexp(rhs, ['\<' v '\.\w+'], 'once'))
            readAt = i;
        end
        if ~isempty(lhs)
            f = lhs{1};
            if readAt > 0 && ~any(strcmp(seen, f))
                bad = true;  badln = i;  badfld = f;  return;
            end
            seen{end+1} = f; %#ok<AGROW>
        end
    end
end
