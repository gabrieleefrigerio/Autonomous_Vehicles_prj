clc; clear all;close all


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

% 3. Creazione della matrice finale nx4
% Ordine standard ROS: [W, X, Y, Z]
Quaternion_Matrix = [quatW, quatX, quatY, quatZ];

Quaternion_Matrix = [waypoints(:, 1:2) Quaternion_Matrix];

%% --- trajectory

N_points = size(Quaternion_Matrix,1) ;
for ii = 1:N_points-1
    % Extract the current waypoint's quaternion components
    currentWaypoint = Quaternion_Matrix(ii, :);
    nextWaypoint  = Quaternion_Matrix(ii+1, :);
    
    % Process the waypoint (e.g., for trajectory planning)
    % (Add your processing logic here)
end