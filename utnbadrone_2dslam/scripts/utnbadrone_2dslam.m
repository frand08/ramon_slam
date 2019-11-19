%% Leo el bag
clc;close all;clear;

% test1.bag tiene:
% -/f_scan: sensor_msgs/LaserScan
% -/imu_data: sensor_msgs/Imu 

% test2.bag tiene:
% -/f_scan: sensor_msgs/LaserScan
% -/imu_data: sensor_msgs/Imu 

% test3.bag tiene:
% -/horizontal_laser_2d: sensor_msgs/MultiEchoLaserScan
% -/imu: sensor_msgs/Imu 

% test4.bag tiene:
% -/b_scan       105 msgs    : sensor_msgs/LaserScan             
% -/clock       1067 msgs    : rosgraph_msgs/Clock               
% -/f_scan       105 msgs    : sensor_msgs/LaserScan             
% -/imu_data     416 msgs    : sensor_msgs/Imu                   
% -/landmark      51 msgs    : cartographer_ros_msgs/LandmarkList
% -/odom_enc     344 msgs    : nav_msgs/Odometry                 
% -/rosout         8 msgs    : rosgraph_msgs/Log

% test5.bag tiene:
% topics:      /f_scan      132 msgs    : sensor_msgs/LaserScan
%              /imu_data    519 msgs    : sensor_msgs/Imu      
%              /odom_enc    427 msgs    : nav_msgs/Odometry    
%              /tf          621 msgs    : tf2_msgs/TFMessage   
%              /tf_static     1 msg     : tf2_msgs/TFMessage

bag = rosbag('test5.bag');
msgs_cell = readMessages(bag);          %% Obtengo todos los mensajes
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
       0 0 0 1 0 ...
      ];

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


% Rototraslaciones de la IMU

imu_to_base_trans = bag.getTransform('base_link','imu_link');
lidar_to_base_trans = bag.getTransform('base_link','front_laser_link');

q_imu = [imu_to_base_trans.Transform.Rotation.W; ...
         imu_to_base_trans.Transform.Rotation.X; ...
         imu_to_base_trans.Transform.Rotation.Y; ...
         imu_to_base_trans.Transform.Rotation.Z];

pos_imu = [imu_to_base_trans.Transform.Translation.X; ...
           imu_to_base_trans.Transform.Translation.Y; ...
           imu_to_base_trans.Transform.Translation.Z]; 

% Hallo rotation matrix c de la IMU     
qw_imu = q_imu(1);
qv_imu = q_imu(2:4);
qv_x_imu = vec3subxoperator(qv_imu);
     
c_imu = (qw_imu*qw_imu-qv_imu'*qv_imu)*eye(3) + ...
          2*(qv_imu*qv_imu') + 2*qw_imu*qv_x_imu;
     
[yaw_imu,roll_imu,pitch_imu] = quat2angle(q_imu');


% Rototraslaciones del LIDAR
       
q_lidar = [lidar_to_base_trans.Transform.Rotation.W; ...
           lidar_to_base_trans.Transform.Rotation.X; ...
           lidar_to_base_trans.Transform.Rotation.Y; ...
           lidar_to_base_trans.Transform.Rotation.Z];

pos_lidar = [lidar_to_base_trans.Transform.Translation.X; ...
             lidar_to_base_trans.Transform.Translation.Y; ...
             lidar_to_base_trans.Transform.Translation.Z]; 

% Hallo rotation matrix c del LIDAR
qw_lidar = q_lidar(1);
qv_lidar = q_lidar(2:4);
qv_x_lidar = vec3subxoperator(qv_lidar);
     
c_lidar = (qw_lidar*qw_lidar-qv_lidar'*qv_lidar)*eye(3) + ...
            2*(qv_lidar*qv_lidar') + 2*qw_lidar*qv_x_lidar;

[yaw_lidar,roll_lidar,pitch_lidar] = quat2angle(q_lidar');


% Necesito:
% 1) Dato de IMU para conocer orientacion inicial e inicializar el KF
init_correction = true;
% 2) Dato de Lidar para inicializar la posicion
init_update = true;
% 3) Lo que venga

%%

while i <= cant_msgs
    % si me llego un dato de IMU
    if strcmp(char(msgs_cell{i}.MessageType),'sensor_msgs/Imu')
        % hacer la transformada (lidar orientation)
        imu_data = msgs_cell{i};

        % separo los datos del mensaje de la IMU
        q = [imu_data.Orientation.W; ...
             imu_data.Orientation.X; ...
             imu_data.Orientation.Y; ...
             imu_data.Orientation.Z];
        
        acc = [imu_data.LinearAcceleration.X; ...
               imu_data.LinearAcceleration.Y; ...
               imu_data.LinearAcceleration.Z];
            
        gyr = [imu_data.AngularVelocity.X; ...
               imu_data.AngularVelocity.Y; ...
               imu_data.AngularVelocity.Z];
        
        % calculo la rotacion del giroscopio y acelerometro para tener una
        % referencia respecto al body frame
        accel = c_imu*acc;
        gyro = c_imu*gyr;
        % obtengo los angulos de Euler
        [yaw,pitch,roll] = quat2angle(q');

        % calculo la rotacion de g para quitarla del dato de aceleracion
        g_rotated = [gravity*(-sin(pitch)); ...
                     gravity*cos(pitch)*sin(roll)];
        
        % El - del accel es porque esta mal el signo de la grabacion, creo
        % (no tiene sentido que z sea positivo)
        acc_data = -accel(1:2);
        
        gyro = gyro(1:2);

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
    
    % si me llego un dato de lidar msgs_cellY YA TUVE UN DATO DE IMU!!...
    elseif (strcmp(char(msgs_cell{i}.MessageType),'sensor_msgs/MultiEchoLaserScan') || ...
           strcmp(char(msgs_cell{i}.MessageType),'sensor_msgs/LaserScan')) && ...
           init_correction == false
    
        scan_data = msgs_cell{i};

        % rangos minimos y maximos del dato del scan actual
        min_range = scan_data.RangeMin;
        max_range = scan_data.RangeMax;

        % cargo los angulos
        angles = scan_data.AngleMin: ...
                 scan_data.AngleIncrement: ... 
                 scan_data.AngleMax;
        
        % cargo los datos de lidar en un vector
        if strcmp(char(bag.MessageList.Topic(i)),'/horizontal_laser_2d')
            ranges = zeros(length(scan_data.Ranges),1);        
            for j = 1:length(scan_data.Ranges)
                % tienen mas de un dato a veces
                scan_data_aux = scan_data.Ranges(j).Echoes(1);
                ranges(j,1) = scan_data_aux;   
            end
        elseif strcmp(char(bag.MessageList.Topic(i)),'/f_scan')
            ranges = scan_data.Ranges;
        end
        
        % obtengo los datos a partir de los angulos y rangos
        lidar_data = lidarScan(ranges,angles);

        % Primero realizo la rigid body transform de los datos del
        % LIDAR
        lidar_body = rotateandtranslate2d(lidar_data.Cartesian', ...
                                          yaw_lidar,pos_lidar(1), ...
                                          pos_lidar(2));
        
        % extraigo contornos
        lidar_data_reg = contourextraction(lidarScan(lidar_body'), ...
                                           Lmin, ...
                                           dist_threshold, ...
                                           min_range, ...
                                           max_range);
        if init_update == true
            % para la primera vez tengo que guardar algun dato de
            % orientacion            
            init_update = false;

            % Roto los puntos con el yaw conseguido en el dato de IMU
            % calculo la transformacion de lo que me da el tita
            lidar_data = rotateandtranslate2d(lidar_body, ...
                                              yaw,0,0);
                                          
            lidar_data_trans_reg = lidarScan(lidar_data');
                                
            map_scans{1,1} = lidar_data_trans_reg;
            map_poses = [0 0 yaw];
            map_poses_reg = [0 0 yaw];
            map = buildMap(map_scans,map_poses,50,100);
            % para calcular velocidad del scan
            time_scan_prev = scan_data.Header.Stamp.Nsec/1e9;
        else
            % para calcular los deltat del scan
            time_scan_reg = scan_data.Header.Stamp.Nsec/1e9;
%             deltat_scan = time_scan_reg - time_scan_prev;
            deltat_scan = 0.25; % ARBITRARIO
            time_scan_prev = time_scan_reg;
            
            % obtengo las poses ultimas de x e y solamente
            x_reg = map_poses(end,1);
            y_reg = map_poses(end,2);
            
            % realizo la fuerza bruta para encontrar el que mejor matchee
            [data,x,y] = bruteforcesearch(map, ...
                                          lidar_data_reg.Cartesian', ...
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
                   vel_y ...
                  ];
               
            % A) Computamos la Ganancia de Kalman            
            
            % Matriz R de Kalmanmsgmsgs_cells_cell
            % REVISAR
            R = eye(4)*deltat_scan*var_lidar;
%             R = eye(2)*deltat_scan*var_lidar;

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
