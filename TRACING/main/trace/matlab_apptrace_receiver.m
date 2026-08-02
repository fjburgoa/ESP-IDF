function matlab_apptrace_receiver()
    clc;
    close all;

    % --- Configuración ---
    host = "127.0.0.1";
    port = 4444;                         % Telnet/control de OpenOCD
    traceFile = fullfile(pwd, "trace.log");
    maxRunTime_s = 60;                   % tiempo total de adquisición
    pollPeriod_s = 0.2;                  % periodo de refresco MATLAB

    % Borra fichero anterior si existe
    if exist(traceFile, 'file')
        delete(traceFile);
    end

    % --- Conecta con OpenOCD (puerto de control) ---
    fprintf("Conectando a OpenOCD en %s:%d...\n", host, port);
    c = tcpclient(host, port, "Timeout", 5);

    pause(0.5);

    % Limpia posibles banners iniciales
    flush(c);

    % --- Lanza captura apptrace en fichero ---
    % OJO: La ruta se interpreta desde OpenOCD.
    cmd = sprintf("esp apptrace start file://%s 1 -1 -1 0 0\n", traceFile);
    write(c, uint8(char(cmd)));

    pause(1.0);

    % --- Prepara figura ---
    figure('Name', 'ESP32-S3 apptrace -> MATLAB', 'NumberTitle', 'off');
    h = plot(nan, nan, '-o');
    grid on;
    xlabel('t [s]');
    ylabel('y');
    title('Recepción de muestras por apptrace');

    t_data = [];
    y_data = [];
    lastPos = 0;

    tStart = tic;

    fprintf("Leyendo trazas desde: %s\n", traceFile);

    while toc(tStart) < maxRunTime_s && ishandle(h)
        if exist(traceFile, 'file')
            fid = fopen(traceFile, 'r');
            if fid ~= -1
                fseek(fid, lastPos, 'bof');
                newText = fread(fid, inf, '*char')';
                lastPos = ftell(fid);
                fclose(fid);

                if ~isempty(newText)
                    lines = splitlines(string(newText));

                    for k = 1:numel(lines)
                        s = strtrim(lines(k));
                        if strlength(s) == 0
                            continue;
                        end

                        parts = split(s, ",");
                        if numel(parts) ~= 2
                            continue;
                        end

                        t = str2double(parts(1));
                        y = str2double(parts(2));

                        if ~isnan(t) && ~isnan(y)
                            t_data(end+1,1) = t; %#ok<AGROW>
                            y_data(end+1,1) = y; %#ok<AGROW>
                        end
                    end

                    set(h, 'XData', t_data, 'YData', y_data);
                    drawnow limitrate;
                end
            end
        end

        pause(pollPeriod_s);
    end

    % --- Para captura ---
    fprintf("Parando apptrace...\n");
    write(c, uint8(sprintf('esp apptrace stop\n')));
    pause(0.5);

    clear c;
    fprintf("Fin.\n");
end

function flush(c)
    pause(0.2);
    n = c.NumBytesAvailable;
    if n > 0
        read(c, n, 'uint8');
    end
end