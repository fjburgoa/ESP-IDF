clear; clc; close all;

%% Archivo de entrada
fname = 'trace.bin';

fid = fopen(fname, 'rb');
if fid < 0
    error('No se pudo abrir %s', fname);
end

raw = fread(fid, inf, 'uint8=>uint8');
fclose(fid);

%% Tamaño de evento
evt_size = 16;   % 4 + 2 + 1 + 1 + 4 + 4

n = floor(numel(raw) / evt_size);
if n == 0
    error('El fichero no contiene eventos completos.');
end

if rem(numel(raw), evt_size) ~= 0
    warning('El tamaño del fichero no es múltiplo de %d bytes. Se truncará el sobrante.', evt_size);
end

raw = raw(1:n*evt_size);

%% Reorganizar en filas de 16 bytes
B = reshape(raw, evt_size, []).';

%% Decodificación little-endian
t_us    = typecast(reshape(B(:,1:4).',  1, []), 'uint32').';
evt     = typecast(reshape(B(:,5:6).',  1, []), 'uint16').';
core    = B(:,7);
prio    = B(:,8);
task_id = typecast(reshape(B(:,9:12).', 1, []), 'uint32').';
aux     = typecast(reshape(B(:,13:16).',1, []), 'uint32').';

%% Tabla
T = table((0:n-1).', t_us, evt, core, prio, task_id, aux, ...
    'VariableNames', {'idx','t_us','evt','core','prio','task_id','aux'});

%% Etiquetas opcionales de evento
evt_name = strings(height(T),1);
evt_name(T.evt == 1) = "SW_IN";
evt_name(T.evt == 2) = "SW_OUT";
evt_name(T.evt == 3) = "TICK";
evt_name(T.evt == 4) = "MARK";
evt_name(evt_name == "") = "UNKNOWN";

T.evt_name = evt_name;

%% Mostrar primeras filas
disp(T(1:min(80,height(T)), :));

%% Guardar CSV legible
writetable(T, 'trace_decoded.csv');

fprintf('Eventos decodificados: %d\n', height(T));
fprintf('CSV generado: trace_decoded.csv\n');