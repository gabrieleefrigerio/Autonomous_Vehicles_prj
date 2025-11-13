%% BAG: select the bag and the /scan topic
clc ; clear all; close all

%%
bagFolder ="/home/diego/Desktop/Assigments/Assignment_1/tb3_nocmd"; % folder with metadata.yaml + *.mcap
bag = ros2bagreader(bagFolder);

% Select a TOPIC (/scan)
sel = select(bag, "Topic", "/odom"); 
sel_scan = select(bag, "Topic", "/scan");
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

% **Array per salvare gli indici (i) per il plottaggio**
turn_left_indices = [];
straight_indices = []; 
turn_right_indices = []; 

for i = 1:N
    odomMsg = odomMsgs{i};
    
    % Posizione
    p = odomMsg.pose.pose.position;
    X(i) = p.x;
    Y(i) = p.y;
    
    % Orientamento (Solo componente z/yaw)
    % ATTENZIONE: se usi ROS/MATLAB, la componente 'z' dell'orientazione 
    % della posa (pose.orientation.z) è la componente Z del quaternione, 
    % NON l'angolo di imbardata (Yaw). Per il Yaw devi convertire il 
    % quaternione completo (x, y, z, w). Ho lasciato la tua riga originale 
    % ma tieni conto di questo se 'theta' non è corretto.
    theta(i) = odomMsg.pose.pose.orientation.z; 
    
    % Twist (Velocità)
    v = odomMsg.twist.twist.linear;
    Vx(i) = v.x;
    Vy(i) = v.y;
    
    w = odomMsg.twist.twist.angular;
    Wz(i) = w.z; % Velocità angolare sull'asse Z (Imbardata)

    % Salvataggio degli indici in base alla velocità angolare (w.z)
    if w.z > 0.01
        % Svolta a SINISTRA (Velocità angolare positiva)
        turn_left_indices = [turn_left_indices; i];
    elseif abs(w.z) < 0.01 % Considero un piccolo margine per 'dritto'
        % Dritto (Velocità angolare vicina a zero)
         straight_indices = [straight_indices; i];
    else % w.z < 0 
        % Svolta a DESTRA (Velocità angolare negativa)
        % Uso `i` per salvare l'indice corrente
        turn_right_indices = [turn_right_indices; i];
    end
    
    % Tempo
    timeMsg = timeMsgs{i};
    % Modificato: Forza la conversione a double
time(i) = double(timeMsg.header.stamp.sec) + double(timeMsg.header.stamp.nanosec) * 1e-9;
end
%% plot data in a timeframe


figure('Name','Traiettoria Odometria XY');
plot(X, Y);
grid on;
title('Traiettoria del Robot');
xlabel('Posizione X');
ylabel('Posizione Y');

%
figure('Name','Velocità Odometria XY');
plot(time,Vx, time,Vy);
legend('V_x', 'V_y');
grid on;
title('velocità del Robot');
xlabel('time');
ylabel('Vels');

figure('Name','Velocità angolare');
plot(time,Wz);
legend('Wz');
grid on;
title('velocità angolare del Robot');
xlabel('time');
ylabel('omega');

figure('Name','posizione angolare');
plot(time,theta);
legend('theta');
grid on;
title('posizione angolare del Robot');
xlabel('time');
ylabel('theta');

%%

figure;
hold on;
title('Traiettoria del Robot Colorata per Tipo di Movimento');
xlabel('Posizione X');
ylabel('Posizione Y');
axis equal; % Assicura che X e Y abbiano la stessa scala

% 1. Plotta le svolte a SINISTRA (es. in blu)
% Sottrae 1 da Y perché in MATLAB gli indici partono da 1 e `turn_left_indices`
% contiene gli indici del vettore `Y`.
plot(X(turn_left_indices), Y(turn_left_indices), 'b.', 'MarkerSize', 8, 'DisplayName', 'Svolta a Sinistra');

% 2. Plotta i tratti DRITTI (es. in verde)
plot(X(straight_indices), Y(straight_indices), 'g.', 'MarkerSize', 8, 'DisplayName', 'Dritto');

% 3. Plotta le svolte a DESTRA (es. in rosso)
plot(X(turn_right_indices), Y(turn_right_indices), 'r.', 'MarkerSize', 8, 'DisplayName', 'Svolta a Destra');
grid on
legend('show');
hold off;


%% CONVERSIONE DAI QUATERNONI PER IL THETA EFFETIVO
% Assumendo che 'odomMsgs', 'timeMsgs' e 'N' siano già definiti

N = length(odomMsgs);
X = zeros(N, 1);
Y = zeros(N, 1);
theta = zeros(N, 1); % Vettore per salvare l'angolo di Yaw (in radianti)

% ... (altre inizializzazioni)
turn_left_indices = [];
straight_indices = []; 
turn_right_indices = []; 

for i = 1:N
    odomMsg = odomMsgs{i};
    
    % --- Calcolo Posizione ---
    p = odomMsg.pose.pose.position;
    X(i) = p.x;
    Y(i) = p.y;
    
    % --- Conversione Quaternione a Yaw ---
    q_x = odomMsg.pose.pose.orientation.x;
    q_y = odomMsg.pose.pose.orientation.y;
    q_z = odomMsg.pose.pose.orientation.z;
    q_w = odomMsg.pose.pose.orientation.w;
    
    % Crea la matrice del quaternione [w x y z] per quat2eul
    quaternion_data = [q_w q_x q_y q_z];
    
    % Converte in angoli di Eulero [Yaw Pitch Roll] (in radianti, sequenza di default 'ZYX')
    % Se i tuoi dati usano un'altra sequenza di rotazione (e.g. 'ZXY'), 
    % devi specificarlo: eul = quat2eul(quaternion_data, 'ZXY');
    eul = quat2eul(quaternion_data); 
    
    % L'angolo di Yaw è il primo elemento se si usa la sequenza 'ZYX' (Z è Yaw, Y è Pitch, X è Roll)
    theta(i) = eul(1); 
    
    
    % --- Calcolo Velocità ---
    v = odomMsg.twist.twist.linear;
    Vx(i) = v.x;
    Vy(i) = v.y;
    % ... (altri calcoli di velocità e tempo)
    
    w = odomMsg.twist.twist.angular;
    Wz(i) = w.z;

    % --- Salvataggio Indici (come nel codice ottimizzato) ---
    if Wz(i) > 0.01
        turn_left_indices = [turn_left_indices; i];
    elseif abs(Wz(i)) < 0.01 
         straight_indices = [straight_indices; i];
    else % Wz(i) < 0 
        turn_right_indices = [turn_right_indices; i];
    end
end


%% 
freq_data = 50;
freq_control = 10;

time_control = time(1) :(1/freq_control): time(end);


Cmd_Vx = interp1(time,Vx,time_control);
Cmd_Omega = interp1(time,Wz,time_control);

time_control = time_control-61.02;
plot(time_control,Cmd_Vx)
plot(time_control, Cmd_Omega)


% Command velocity (geometry_msgs/Twist): simple publisher
[velPub, velMsg] = ros2publisher(node,"/cmd_vel","geometry_msgs/TwistStamped");
disp('Sending cmd_vel for ~3 seconds...');
t0 = tic; last = tic; period = 0.1;

while toc(t0) < time_control(end)
    % -- Fill twist --
    index = max(1, min(length(Cmd_Vx), round(toc(t0) / period) + 1));
    velMsg.twist.linear.x = Cmd_Vx(index) ; % m/s
    velMsg.twist.angular.z = Cmd_Omega(index); % rad/s
    % -- (Good practice) set header.stamp to "now" --
    t = datetime('now','TimeZone','UTC');
    s = posixtime(t); 
    velMsg.header.stamp.sec = int32(floor(s));
    velMsg.header.stamp.nanosec = uint32(round((s - floor(s))*1e9));
    send(velPub, velMsg);
    elapsed = toc(last); pause(max(0, period - elapsed)); last = tic;
end

%% map reconstruction 

scanMsgs = readMessages(sel_scan);
N = length(scanMsgs);
%R = rotz(theta);

sel_time_scan = select(sel_scan, "Time", [t0 t1]);
timeMsgs_scan = readMessages(sel_time_scan);
 


for i = 1:N % per leggere il tempo
    timeMsg_scan = timeMsgs_scan{i};
    % Modificato: Forza la conversione a double
time_scan(i) = double(timeMsg_scan.header.stamp.sec) + double(timeMsg_scan.header.stamp.nanosec) * 1e-9;
end


theta_scan = interp1(time,theta,time_scan);
pos_scan_x = interp1(time,X,time_scan);
pos_scan_y = interp1(time,Y,time_scan);
 %%
figure()
for i = 1:N
    ls = scanMsgs{i}; % i-th message
    sc= rosReadLidarScan(ls); %reading messages
    Cartesian = [sc.Cartesian , zeros(360,1)]*rotz(theta_scan(i));
    x_map = Cartesian(:,1) + pos_scan_x(i);
    x_valid(:,i)= x_map(isfinite(x_map));
    y_map= Cartesian(:,2) + pos_scan_y(i);
    y_valid(:,i) = y_map(isfinite(y_map));
   plot(x_valid(i),y_valid(i))
   hold on
   grid on
end
%%
% BEFORE the loop, initialize as cell arrays instead of matrices
x_valid = cell(1, N);
y_valid = cell(1, N);

figure()
for i = 1:N
    ls = scanMsgs{i}; % i-th message
    sc= rosReadLidarScan(ls); %reading messages
    Cartesian = rotz(rad2deg(theta_scan(i)))*[sc.Cartesian , zeros(360,1)]';
    x_map = Cartesian(:,1) + pos_scan_x(i);
    y_map= Cartesian(:,2) + pos_scan_y(i); % Moved y_map calculation up

    % Store the resulting vectors into the i-th cell
    x_valid{i} = x_map(isfinite(x_map)); 
    y_valid{i} = y_map(isfinite(y_map));
    
    % Plotting needs to change to access the cell content with {}
    plot(x_valid{i}, y_valid{i}, '.') 
    hold on
    grid on
end
%%
figure()
for i = 1:786
    ls = scanMsgs{i}; % i-th message
    sc= rosReadLidarScan(ls); %reading messages
  Cartesian = (rotz(rad2deg(theta_scan(i)))*[sc.Cartesian , zeros(360,1)]')';
    x_map = Cartesian(:,1) + pos_scan_x(i);
    y_map= Cartesian(:,2) + pos_scan_y(i);
    
    % Get the indices of the finite points once
    finite_idx = isfinite(x_map) & isfinite(y_map);
    
    % Plot only the finite points directly
    plot(x_map(finite_idx), y_map(finite_idx), '.')
    axis equal
    hold on
    grid on
end
% 1. Plotta le svolte a SINISTRA (es. in blu)
% Sottrae 1 da Y perché in MATLAB gli indici partono da 1 e `turn_left_indices`
% contiene gli indici del vettore `Y`.
plot(X(turn_left_indices), Y(turn_left_indices), 'b.', 'MarkerSize', 8, 'DisplayName', 'Svolta a Sinistra');

% 2. Plotta i tratti DRITTI (es. in verde)
plot(X(straight_indices), Y(straight_indices), 'g.', 'MarkerSize', 8, 'DisplayName', 'Dritto');

% 3. Plotta le svolte a DESTRA (es. in rosso)
plot(X(turn_right_indices), Y(turn_right_indices), 'r.', 'MarkerSize', 8, 'DisplayName', 'Svolta a Destra');
hold on