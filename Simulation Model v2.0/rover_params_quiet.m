function p = rover_params_quiet()
%ROVER_PARAMS_QUIET  rover_params() sin la impresion de verificacion.
%   La verificacion usa fprintf y warning, que no pueden vivir dentro de un
%   bloque MATLAB Function ni ejecutarse 600 000 veces por corrida. Esta
%   envoltura silencia la salida sin duplicar un solo valor.
    ws = warning('off', 'all');
    ev = evalc('p = rover_params();');  %#ok<NASGU>
    warning(ws);
end
