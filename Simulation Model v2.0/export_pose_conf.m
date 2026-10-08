function export_pose_conf(fname)
%EXPORT_POSE_CONF  Escribe los parametros del nucleo para la aplicacion CMAES.
%
%   EXPORT_POSE_CONF()        escribe ./codegen/pose_model.conf
%   EXPORT_POSE_CONF(fname)   escribe en fname
%
%   El modelo es la UNICA fuente de los parametros (DESIGN.md, seccion 12).
%   Este archivo los vuelca en texto "clave = valor" con las mismas claves
%   que los structs que recibe el codigo generado (ver codegen_hlc_core):
%
%       geo.<campo>   sep_geo_t           (sep_geo_params)
%       ag.<campo>    sep_agent_params_t  (sim_params().agents)
%       prm.<campo>   sep_ekf_params_t    (sep_ekf_params)
%
%   La aplicacion lee este bloque al arrancar; recalibrar es editar el
%   archivo de parametros del modelo, exportar de nuevo y reiniciar, sin
%   regenerar ni recompilar. Los valores van con 17 cifras significativas,
%   asi que el C y MATLAB usan exactamente los mismos doubles.
%
%   Cada linea lleva el estado del valor (MED, DER, PROV, TBD, DES) cuando
%   rover_params lo conoce.

    if nargin < 1 || isempty(fname)
        d = fullfile(fileparts(mfilename('fullpath')), 'codegen');
        if ~exist(d, 'dir'), mkdir(d); end
        fname = fullfile(d, 'pose_model.conf');
    end

    p   = rover_params_quiet();
    geo = sep_geo_params();
    prm = sep_ekf_params();
    ag  = sim_params();  ag = ag.agents;

    gk = {'m_per_tick_R','m_per_tick_L','B_eff','stop_ticks'};
    ak = {'gyro_scale','dt_reject','dcount_reject','loop_ms','reset_window', ...
          'k_reref','n_still','tilt_rollover','t_gap_max'};
    pk = fieldnames(prm).';

    fid = fopen(fname, 'w');
    assert(fid > 0, 'no se pudo abrir %s', fname);
    c = onCleanup(@() fclose(fid));

    fprintf(fid, '# pose_model.conf - parametros del nucleo de estimacion\n');
    fprintf(fid, '# Generado por export_pose_conf.m el %s. NO editar a mano:\n', ...
            char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));
    fprintf(fid, '# editar sep_geo_params / sep_ekf_params / sim_params y exportar.\n');
    fprintf(fid, '# Formato: clave = valor [valor ...]   # estado\n\n');

    local_block(fid, 'geo', geo, gk, p.status);
    local_block(fid, 'ag',  ag,  ak, p.status);
    local_block(fid, 'prm', prm, pk, p.status);

    fprintf('Parametros exportados a %s\n', fname);
end

% ---------------------------------------------------------------------
function local_block(fid, pre, s, keys, status)
    fprintf(fid, '# --- %s ---\n', pre);
    for i = 1:numel(keys)
        k = keys{i};
        v = s.(k);
        txt = strjoin(arrayfun(@(x) sprintf('%.17g', x), v(:).', ...
                      'UniformOutput', false), ' ');
        if isfield(status, k)
            fprintf(fid, '%s.%s = %s   # %s\n', pre, k, txt, status.(k));
        else
            fprintf(fid, '%s.%s = %s\n', pre, k, txt);
        end
    end
    fprintf(fid, '\n');
end
