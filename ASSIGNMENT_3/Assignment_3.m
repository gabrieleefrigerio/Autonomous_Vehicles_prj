% Inizializzazione della matrice dati

% Data structure: {Nome_Colore_File, X, Y}
sphere_data = {
    'red',              0, 3;
    'big_purple',     6,  0;
    'big_red',         -6,  0;
   };
% --- Parametri fissi ---
worldName = 'default'; % Sostituisci se necessario
z = 0.4;               % La coordinata Z è costante

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
        % Potrebbe essere utile aggiungere un 'break' o 'continue' qui in caso di errore
    else
        disp("  -> Spawn riuscito.");
    end
end

disp('--- Importazione completata ---');



%%

Kv = 0.8;
Kw = 0.3;
toll = 0.001;
theta_ref = 1.8*pi;