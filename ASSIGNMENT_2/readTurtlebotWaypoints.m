function [positionTS, orientationTS] = readTurtlebotWaypoints(filename)
% READTURTLEBOTWAYPOINTS Legge un file TXT con waypoint (X, Y, Theta) e li
% converte in timeseries (TS) di Posizione e Quaternioni per Simulink.
%
% INPUT:
%   filename - Nome del file TXT contenente i dati.
%
% OUTPUT:
%   positionTS - Timeseries (X, Y) per la posizione.
%   orientationTS - Timeseries (W, X, Y, Z) per l'orientamento (Quaternioni).

% Verifica se il file esiste
if ~exist(filename, 'file')
    error('File non trovato: %s', filename);
end

% Legge il contenuto del file come testo
fileContent = fileread(filename);

% Pulisce il contenuto: rimuove intestazioni, commenti e righe vuote.
% Cerchiamo solo le righe che iniziano con un numero o con 'init'.
lines = strsplit(fileContent, '\n');
dataLines = {};
for i = 1:length(lines)
    line = strtrim(lines{i}); % Rimuove spazi iniziali/finali
    
    % Ignora righe troppo corte, commenti ('#'), e la riga di intestazione ('x y theta')
    if isempty(line) || startsWith(line, '#') || contains(lower(line), 'theta')
        continue;
    end
    
    % Sostituisce i separatori con spazi e pulisce la riga
    line = strrep(line, ';', ' ');
    
    % Se è la riga 'init', la trasformiamo in '0 ' per il parsing
    if startsWith(line, 'init', 'IgnoreCase', true)
        line = strrep(line, 'init', '0');
    end
    
    % Rimuove il numero di indice iniziale (es. '1 ', '2 ')
    parts = strsplit(line, ' ', 'CollapseDelimiters', true);
    if length(parts) >= 4 % Deve esserci Indice, X, Y, Theta
         % Lasciamo solo X, Y, Theta. Concateniamo eventuali parti di Theta (es. 'pi/2')
         dataLines{end+1} = strjoin(parts(2:4), ' '); 
    elseif length(parts) == 3 % Se la riga era già pulita e ha solo X, Y, Theta
         dataLines{end+1} = line;
    end
end

% Unisce le righe di dati pulite
cleanData = strjoin(dataLines, '\n');

% Valuta le espressioni che contengono 'pi' (es. 'pi/2', '5*pi/4')
% Questo è NECESSARIO perché readmatrix non gestisce 'pi' come simbolo matematico.
waypoints = [];
for i = 1:length(dataLines)
    parts = strsplit(dataLines{i}, ' ', 'CollapseDelimiters', true);
    if length(parts) == 3
        try
            X = str2double(parts{1});
            Y = str2double(parts{2});
            % Valuta l'espressione di Theta (es. 'pi/2')
            Theta = evalin('base', parts{3}); 
            waypoints = [waypoints; X, Y, Theta];
        catch
            warning('Impossibile valutare la riga di dati: %s', dataLines{i});
        end
    end
end

if isempty(waypoints)
    error('Nessun dato valido X, Y, Theta trovato nel file.');
end

% --- Elaborazione dei Dati per Simulink ---

% Vettore temporale (assume che ogni punto venga mantenuto per un istante)
time = (0:size(waypoints, 1) - 1)'; 

X = waypoints(:, 1);
Y = waypoints(:, 2);
Theta = waypoints(:, 3);

% 1. Conversione Theta (Yaw) in Quaternione (W, X, Y, Z)
% Yaw (theta) avviene attorno all'asse Z (X=0, Y=0).
quatW = cos(Theta / 2);
quatZ = sin(Theta / 2);
quatX = zeros(size(Theta));
quatY = zeros(size(Theta));

% 2. Creazione delle Strutture Timeseries
% Posizione: [X, Y]
positionData = [X, Y];
positionTS = timeseries(positionData, time, 'Name', 'WaypointsPosition');

% Orientamento: [W, X, Y, Z]
orientationData = [quatW, quatX, quatY, quatZ];
orientationTS = timeseries(orientationData, time, 'Name', 'WaypointsOrientation');

end