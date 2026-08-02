clear; clc; close all;

T = readtable('trace_decoded.csv');

%% Eventos
EVT_SW_IN  = 1;
EVT_SW_OUT = 2;

%% Quedarse solo con cambios de contexto
S = T(T.evt == EVT_SW_IN | T.evt == EVT_SW_OUT, :);

%% Ignorar task_id = 0 (idle o desconocida)
S = S(S.task_id > 0, :);

%% Lista de tareas presentes
task_ids = unique(S.task_id);

%% Reconstruir segmentos [task_id, t_ini_us, t_fin_us]
segments = [];

for i = 1:length(task_ids)
    tid = task_ids(i);
    X = S(S.task_id == tid, :);

    t_in  = X.t_us(X.evt == EVT_SW_IN);
    t_out = X.t_us(X.evt == EVT_SW_OUT);

    n = min(length(t_in), length(t_out));
    if n == 0
        continue;
    end

    t_in  = t_in(1:n);
    t_out = t_out(1:n);

    valid = t_out > t_in;

    seg = [ ...
        repmat(tid, sum(valid), 1), ...
        t_in(valid), ...
        t_out(valid) ...
    ];

    segments = [segments; seg];
end

if isempty(segments)
    error('No se han encontrado ejecuciones válidas.');
end

%% Tabla de segmentos
Seg = array2table(segments, ...
    'VariableNames', {'task_id','t_ini_us','t_fin_us'});

%% Representación temporal simple
figure;
hold on;

num_tasks = length(task_ids);
colors = lines(num_tasks);

% límites de tiempo
t0 = min(Seg.t_ini_us) / 1000;
tf = max(Seg.t_fin_us) / 1000;

for i = 1:num_tasks
    tid = task_ids(i);
    Xi = Seg(Seg.task_id == tid, :);

    y = i;

    % --- línea base ---
    plot([t0 tf], [y y], '--', 'Color', [0.7 0.7 0.7]);

    % --- segmentos ---
    for k = 1:height(Xi)
        x = Xi.t_ini_us(k) / 1000;                     % ms
        w = (Xi.t_fin_us(k) - Xi.t_ini_us(k)) / 1000; % ms

        rectangle('Position', [x, y-0.35, w, 0.7], ...
                  'FaceColor', colors(i,:), ...
                  'EdgeColor', 'k');
    end
end

xlabel('Tiempo [ms]');
ylabel('Tarea');
title('Ejecuciones de tareas');

yticks(1:num_tasks);
yticklabels(compose('Task %d', task_ids));

grid on;
hold off;

xlabel('Tiempo [ms]');
ylabel('Tarea');
title('Ejecuciones de tareas');
yticks(1:length(task_ids));
yticklabels(compose('Task %d', task_ids));
grid on;
hold off;