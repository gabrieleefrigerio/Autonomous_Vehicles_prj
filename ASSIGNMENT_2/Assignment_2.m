clc; clear all;close all
%%

% 1. Definizione dei Waypoint
% La matrice 'waypoints' contiene [X, Y, Theta_radianti]
% Ho usato la funzione 'eval' per calcolare le espressioni con pi greco.

tol = 0.05;

waypoints = [
    % X;   Y;    Theta
  5   0    pi/2;   % 1
  5   5    5*pi/4; % 2
 -5  -5    pi/2;   % 3
 -5   5    0;      % 4
  0   0    0;      % 5
  3   3    3*pi/4; % 6
 -3   0    3*pi/2; % 7
  0  -3    pi/4;   % 8
  3   0    pi/2;   % 9
  0   0    3*pi/2  % 10
   ];

% Assumiamo che 'Theta' sia un vettore colonna contenente tutti gli angoli:
Theta = waypoints(:,3);

% 1. Calcolo delle componenti del Quaternione (W e Z)
% Rotazione 2D (Yaw) attorno all'asse Z (Quaternione: W, X, Y, Z)
quatW = cos(Theta / 2);
quatZ = sin(Theta / 2);

% 2. Le componenti X e Y sono zero per la rotazione sul piano
quatX = zeros(size(Theta));
quatY = zeros(size(Theta));

Angl = zeros(3,10)';
Angl(:,3) = waypoints(:,3);

Quat = eul2quat(Angl,"XYZ");

% 3. Creazione della matrice finale nx4
%Ordine per ROS 2: [X, Y, Z, W]
Quaternion_Matrix = [quatX, quatY, quatZ, quatW];

Quaternion_Matrix = [waypoints(:, 1:2) Quaternion_Matrix];





%% control
Kv = 0.8;
Kw = 2;
% index = 1;
%angular_error = 1;



 
%% --- trajectory

N_points = size(Quaternion_Matrix,1) ;
for ii = 1:N_points-1
    % Extract the current waypoint's quaternion components
    currentWaypoint = Quaternion_Matrix(ii, :);
    nextWaypoint  = Quaternion_Matrix(ii+1, :);
    
    % Process the waypoint (e.g., for trajectory planning)
    % (Add your processing logic here)
end


%%
bagFolder ="/home/nicola-bertocchi/Desktop/Autonomous_Vehicles/Assignments/ASSIGNMENT_2/tb3_subset"; % folder with metadata.yaml + *.mcap
bag = ros2bagreader(bagFolder);

% Select a TOPIC (/scan)
sel = select(bag, "Topic", "/odom"); 

%%

% (additional) Limitin time:
 t0 = sel.StartTime ; t1 = sel.EndTime ; 

sel_time = select(sel, "Time", [t0 t1]); 

% read messages as struct
odomMsgs = readMessages(sel);
timeMsgs= readMessages(sel_time);
%%

N = length(odomMsgs);
X = zeros(N, 1);
Y = zeros(N, 1);
theta = zeros(N, 1); 
Vx = zeros(N, 1);
Vy = zeros(N, 1);
Wz = zeros(N, 1);
time = zeros(N, 1);



for i = 1:N
    odomMsg = odomMsgs{i};
    
    % Posizione
    p = odomMsg.pose.pose.position;
    X(i) = p.x;
    Y(i) = p.y;
    

    % Tempo
    timeMsg = timeMsgs{i};
    % Modificato: Forza la conversione a double
time(i) = double(timeMsg.header.stamp.sec) + double(timeMsg.header.stamp.nanosec) * 1e-9;
end
%% --- Sezione Plot Animata  ---

figure('Name','Traiettoria Odometria XY');

% 1. Imposta la figura e gli assi (importante farlo prima)
grid on;
title('Traiettoria del Robot (Animata)');
xlabel('Posizione X');
ylabel('Posizione Y');
axis equal; 
axis([min(X)-1 max(X)+1 min(Y)-1 max(Y)+1]); 

% 2. Crea un oggetto linea animata
h = animatedline('Color', 'b', 'LineWidth', 2);

% 3. Ciclo di animazione VELOCE

for i = 1:N
    % Aggiungi il nuovo punto alla linea
    addpoints(h, X(i), Y(i));
    
    % Aggiorna la finestra grafica in modo efficiente
     drawnow limitrate; 
end
% 4. (Importante) Assicurati di disegnare l'ultimo punto finale
addpoints(h, X(N), Y(N));
 drawnow;