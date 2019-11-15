%% Leo el bag
clc;close all;clear;
% bag = rosbag('_2019-10-14-10-38-58_pieza_fondo_1hz.bag');
% bag = rosbag('_2019-10-15-15-19-10_pieza_fondo_1hz.bag');
bag = rosbag('test3.bag');
msgs_cell = readMessages(bag);      %% Obtengo todos los mensajes
cant_msgs = length(msgs_cell);

%% Lineal Kalman Filter (tomo pos x e y, vel x e y, y yaw)

Lmin = 10;              % minima cantidad de datos de un contour
dist_threshold = 0.1;   % distancia minima entre puntos consecutivos

var_lidar = 0.1;        % REVISAR

gravity = -9.81;        % REVISAR

i = 2;

% Me indica si vengo de un update de EKF o si no
ekf_update = false;

% Matriz H de Kalman
H_k = [1 0 0 0 0; ...
       0 1 0 0 0; ...
       0 0 1 0 0; ...
       0 0 0 1 0];

% desvios estandar y varianzas de gyro y acc
% REVISAR
sigma_gyrx = 0.1;
sigma_gyry = 0.1;
sigma_accx = 0.1;
sigma_accy = 0.1;

sigma_tita = 0.1;      % cambiarlo!

var_gyrx = sigma_gyrx*sigma_gyrx;
var_gyry = sigma_gyry*sigma_gyry;
var_accx = sigma_accx*sigma_accx;
var_accy = sigma_accy*sigma_accy;

var_tita = sigma_tita*sigma_tita;

% Necesito:
% 1) Dato de IMU para conocer orientacion inicial e inicializar el ES-EKF
init_correction = true;
% 2) Dato de Lidar para inicializar la posicion
init_update = true;
% 3) Lo que venga

while i <= cant_msgs
    % si me llego un dato de IMU
    if strcmp(char(bag.MessageList.Topic(i)),'/imu')
        % hacer la transformada (lidar orientation)
        imu_data = msgs_cell{i};

        % separo los datos del mensaje de la IMU
        q = [imu_data.Orientation.W; ...
             imu_data.Orientation.X; ...
             imu_data.Orientation.Y; ...
             imu_data.Orientation.Z];
        
        accel = [imu_data.LinearAcceleration.X; ...
                 imu_data.LinearAcceleration.Y];
            
        gyro = [imu_data.AngularVelocity.X; ...
                imu_data.AngularVelocity.Y];
        
        % obtengo los angulos de Euler
        [yaw,pitch,roll] = quat2angle(q');

        % calculo la rotacion de g para quitarla del dato de aceleracion
        g_rotated = [gravity*(-sin(pitch)); ...
                     gravity*cos(pitch)*sin(roll)];
        
        % El - del accel es porque esta mal el signo de la grabacion, creo
        % (no tiene sentido que z sea positivo)
        acc_data = -accel;

        % calculo la aceleracion quitando el efecto de la gravedad
        acc_nogravity = acc_data - g_rotated;

        % cargo el valor del tiempo actual
        time_reg = imu_data.Header.Stamp.Nsec/1e9;
        
        % inicializo en caso que caiga por primera vez, sino calculo el
        % correction
        
        % con esto me aseguro que tenga por lo menos un dato de yaw para el
        % dato de lidar subsiguiente
        if init_correction == true
            init_correction = false;
            P_k = 100*eye(5); % Cuanto tendria que valer P?
        end
        
        % si no tomo un dato de lidar, lo dejo todo como estaba
        if init_update == true
            tita_k = yaw;
            u_k = [acc_nogravity; 0; 0; 0];
            pos_k = [0;0];
            vel_k = [0;0];
        
        % si el dato de lidar ya fue computado, hago el correction step
        else
            % cargo los datos en los vectores de Kalman
            tita_k1 = yaw;
            u_k1 = [acc_nogravity; 0; 0; 0];
            
            % cargo los estados previos en el anterior (k-1)
            pos_k1 = pos_k;
            vel_k1 = vel_k;
            P_k1 = P_k;
            
            % 1) Actualizamos el estado con entradas de la IMU, basados en
            % nuestro estado anterior
            deltat = time_reg - time_prev;
            
            F_k1 = [1 0 deltat   0    0; ...
                    0 1   0    deltat 0; ...
                    0 0   1      0    0; ...
                    0 0   0      1    0; ...
                    0 0   0      0    1];
            
            aux = deltat*deltat/2;
            G_k1 = [  aux      0    0  0  0; ...
                       0      aux   0  0  0; ...
                     deltat    0    0  0  0; ...
                       0     deltat 0  0  0; ...
                       0       0    0  0  0];
             
            % Obtengo posiciones, velocidades y tita para k 
            out = F_k1*[pos_k1;vel_k1;tita_k1] + G_k1*u_k1;
            pos_k = [out(1);out(2)];
            vel_k = [out(3);out(4)];
            tita_k = out(5);
            
%            pos_k = pos_k1 + deltat*vel_k1 + (deltat*deltat/2)*u_k1;
                   
%            vel_k = vel_k1 + deltat*u_k1;
            
%            tita_k = tita_k1;
            
            % 2) Propagamos incertidumbres            
            q_accx = deltat*deltat*var_accx;
            q_gyrx = deltat*deltat*var_gyrx;          
            q_accy = deltat*deltat*var_accy;
            q_gyry = deltat*deltat*var_gyry;
            q_tita = deltat*deltat*var_tita; % cambiarlo!
            Q_k1 = [q_accx    0     0      0      0   ; ...
                      0    q_accy   0      0      0   ; ...
                      0       0   q_gyrx   0      0   ; ...
                      0       0     0    q_gyry   0   ; ...
                      0       0     0      0    q_tita];

            P_k = F_k1*P_k1*F_k1' + Q_k1;
        end
        time_prev = time_reg;        
    
    % si me llego un dato de lidar Y YA TUVE UN DATO DE IMU!!...
    elseif strcmp(char(bag.MessageList.Topic(i)),'/horizontal_laser_2d') ...
           && init_correction == false
    
        scan_data = msgs_cell{i};

        % rangos minimos y maximos del dato del scan actual
        min_range = scan_data.RangeMin;
        max_range = scan_data.RangeMax;

        % cargo los datos de lidar en un vector
        ranges = zeros(length(scan_data.Ranges),1);        
        for j = 1:length(scan_data.Ranges)
            % tienen mas de un dato a veces
            scan_data_aux = scan_data.Ranges(j).Echoes(1);
            ranges(j,1) = scan_data_aux;   
        end
        
        % cargo los angulos
        angles = scan_data.AngleMin: ...
                 scan_data.AngleIncrement: ... 
                 scan_data.AngleMax;
        
        % obtengo los datos a partir de los angulos y rangos
        lidar_data = lidarScan(ranges,angles);

        % extraigo contornos
        lidar_data_reg = contourextraction(lidar_data, ...
                                           Lmin, ...
                                           dist_threshold, ...
                                           min_range, ...
                                           max_range);
        if init_update == true
            % para la primera vez tengo que guardar algun dato de
            % orientacion            
            init_update = false;

            % calculo la transformacion de lo que me da el tita
            lidar_data_trans_reg = lidarScan(rotateandtranslate2d( ...
                                    lidar_data_reg.Cartesian', ...
                                    yaw,0,0)');
                                
            map_scans{1,1} = lidar_data_trans_reg;
            map_poses = [0 0 yaw];
            map_poses_reg = [0 0 yaw];
            map = buildMap(map_scans,map_poses,50,100);
            % para calcular velocidad del scan
            time_scan_prev = scan_data.Header.Stamp.Nsec/1e9;
        else
            % para calcular los deltat del scan
            time_scan_reg = scan_data.Header.Stamp.Nsec/1e9;
            deltat_scan = time_scan_reg - time_scan_prev;
            time_scan_prev = time_scan_reg;
            
            % obtengo las poses ultimas de x e y solamente
            x_reg = map_poses(end,1);
            y_reg = map_poses(end,2);
            
            % realizo la fuerza bruta para encontrar el que mejor matchee
            [data,x,y] = bruteforcesearch(map, ...
                                          lidar_data_reg, ...
                                          pos_k(1), ...
                                          pos_k(2), ...
                                          tita_k, ...
                                          max_range);
       
            % calculo la velocidad a partir del dato del lidar
            delta_x = x_reg - x;
            delta_y = y_reg - y;
            vel_x = delta_x/deltat_scan;
            vel_y = delta_y/deltat_scan;

            % data del lidar
            y_k = [x; ...
                   y; ...
                   vel_x; ...
                   vel_y];
            
            % A) Computamos la Ganancia de Kalman            
            
            % Matriz R de Kalman
            % REVISAR
            R = eye(4)*deltat_scan*var_lidar;

            K_k = P_k*H_k'*(H_k*P_k*H_k'+ R)^(-1);

            % B) Corregimos el estado predecido
            out = [pos_k;vel_k;tita_k] + ...
                                   K_k*(y_k-H_k*[pos_k;vel_k;tita_k]);
            pos_k = [out(1);out(2)];
            vel_k = [out(3);out(4)];
            tita_k = out(5);
            
            % C) Computamos la covarianza corregida
            P_k = (1 - K_k*H_k)*P_k;
            
            map_poses_reg = [pos_k(1) pos_k(2) tita_k];
            map_poses = [map_poses; map_poses_reg];
            map_scans{end+1,1} = lidar_data_reg;
            map = buildMap(map_scans,map_poses,50,100);            
        end
    end
    i = i + 1;
end
