worldName = 'default';

% Data structure: {Nome_Colore_File, X, Y}
sphere_data = {
    'red',            0,  1;
    'big_purple',     3,  0;
    'big_red',       -3,  0;
   };
% --- Parametri fissi ---
 % Sostituisci se necessario
z = 0.4; 
tutto_ok = true;

% --- Ciclo di importazione ---
N_spheres = size(sphere_data, 1); 

disp(['Inizio importazione di ', num2str(N_spheres), ' sfere, una per colore...']);

for i = 1:N_spheres
    % 1. Estrazione dati dal cell array
    color_name = sphere_data{i, 1}; % Nome del colore per il file SDF
    x = sphere_data{i, 2};          % Coordinata X
    y = sphere_data{i, 3};          % Coordinata Y
    
    % 2. Costruzione dinamica del percorso del file SDF
    % Esempio: 'bright_green_sphere.sdf'
    modelFileName = [color_name, '_sphere.sdf'];
    modelPath = fullfile(pwd, modelFileName);

    if exist(modelPath, 'file') ~= 2
        fprintf(2, 'ERRORE: Il file "%s" NON ESISTE in:\n%s\n', modelFileName, pwd);
        disp('--> Salto allo spawn successivo...');
        tutto_ok = false;
        continue; % Salta questa sfera e passa alla prossima
    end
    
    % 3. Nome univoco della sfera nel simulatore
    % Esempio: 'ball_bright_green_1'
    model_name = ['ball_', color_name, '_', num2str(i)];
    
    % 4. Costruzione del comando ROS/Gazebo
    cmd = sprintf('unset LD_LIBRARY_PATH; source /opt/ros/jazzy/setup.bash; ros2 run ros_gz_sim create -world %s -file %s -name %s -x %.2f -y %.2f -z %.2f', ...
        worldName, modelPath, model_name, x, y, z);
    
    % 5. Esecuzione del comando
    disp(['Spawn sfera ', num2str(i), ' (', model_name, ') da file: ', modelFileName]);
    [status, out] = system(['bash -c "' cmd '"']);
    
    % 6. Gestione degli errori
    if status ~= 0
        disp("  -> Spawn FALLITO:");
        disp(out);
    else
        disp("  -> Spawn riuscito.");
    end
end

if tutto_ok
    disp('--- Importazione completata ---');
else
    fprintf(2, 'ATTENZIONE: Ci sono stati degli errori. Importazione NON completata del tutto.');
end

%%
Kv = 0.8;
Kw = 0.3;
toll = 0.001;
theta_ref = 0.6*pi;

%% Colors Limit Definition
hexCode_1 = '#c83030';
hexCode_2 = '#c80067';

rgb_1 = validatecolor(hexCode_1);
rgb_2 = validatecolor(hexCode_2);

hsv_1 = rgb2hsv(rgb_1);
hsv_2 = rgb2hsv(rgb_2);

limit_sup = zeros(1,length(hsv_1));
for i=1:length(hsv_1)
    if i == 2
        limit_sup(i) = 1;
    else
        limit_sup(i) = hsv_1(i) + 0.1;
    end
end

limit_inf = zeros(1,length(hsv_1));
for i=1:length(hsv_1)
    if i == 2
        limit_inf(i) = 0.35;
    elseif hsv_1(i)<0.1
        limit_inf(i) = 1 + (hsv_1(i) - 0.1);
    else
        limit_inf(i) = hsv_1(i) - 0.2; 
    end
end


