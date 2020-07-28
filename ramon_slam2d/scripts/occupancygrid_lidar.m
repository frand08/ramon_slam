%% Leo el bag
clc;close all;clear;
% bag = rosbag('_2019-10-14-10-38-58_pieza_fondo_1hz.bag');
% bag = rosbag('_2019-10-15-15-19-10_pieza_fondo_1hz.bag');
bag = rosbag('test3.bag');
msgs_cell = readMessages(bag);      %% Obtengo todos los mensajes
cant_msgs = length(msgs_cell);

%% Separo scans del resto
%{
finish = 0;
j = 1;
scans = {};
for i=2:1:cant_msgs
    if strcmp(char(bag.MessageList.Topic(i)),'/scan')
        sample = msgs_cell{i,1};
        finish = 1;
        ranges = msgs_cell{i,1}.Ranges;
        angles = msgs_cell{i,1}.AngleMin: ...
                          msgs_cell{i,1}.AngleIncrement: ...
                          msgs_cell{i,1}.AngleMax;
        max_range = msgs_cell{i,1}.RangeMax;
        min_range = msgs_cell{i,1}.RangeMin;
        scans{j,1} = lidarScan(ranges, angles);
        j = j + 1;
    end
end
%}

%% Para los test

finish = 0;
scan_index = 0;
scan_data = {};
imu_index = 0;
imu_data = {};
print_occgrid = 0;
i = 2;

map_scans = {};
map_poses = [];
map_count = 0;
start_scans = 0;
imu_first_time = 1;
gravity = -9.81; % COMO LO PUEDO OBTENER DE LOS DATOS, O QUE ?!?!??!?!?

% poner datos de gyr y acc
sigma_gyr = 0.01;
sigma_acc = 0.01;

% Para saber si pase por la etapa de update o no
ekf_update = 0;

while i <= cant_msgs
    if strcmp(char(bag.MessageList.Topic(i)),'/imu')
        % hacer la transformada (lidar orientation)
        imu_index = imu_index + 1;
        imu_data(imu_index,1) = msgs_cell(i);
        start_scans = 1;
        
        q_k = [imu_data{imu_index,1}.Orientation.W; ...
               imu_data{imu_index,1}.Orientation.X; ...
               imu_data{imu_index,1}.Orientation.Y; ...
               imu_data{imu_index,1}.Orientation.Z];
        accel = [imu_data{imu_index,1}.LinearAcceleration.X; ...
                 imu_data{imu_index,1}.LinearAcceleration.Y; ...
                 imu_data{imu_index,1}.LinearAcceleration.Z];
        gyro = [imu_data{imu_index,1}.AngularVelocity.X; ...
                imu_data{imu_index,1}.AngularVelocity.Y; ...
                imu_data{imu_index,1}.AngularVelocity.Z];
        
        [yaw,pitch,roll] = quat2angle(q_k');
        
        g_rotated = [gravity*(-sin(pitch)); ...
                     gravity*cos(pitch)*sin(roll); ...
                     gravity*cos(pitch)*cos(roll)];
        
        acc_data = -accel;
        % El - del accel es porque esta mal el signo de la grabacion, creo
        % (no tiene sentido que z sea positivo)
        acc_nogravity = acc_data - g_rotated;

        time_reg = imu_data{imu_index,1}.Header.Stamp.Nsec/1e9;
        
        if imu_first_time == 1
            % Solo cargo condiciones iniciales
            imu_first_time = 0;
            pos_pred_k = [0;0;0];
            vel_pred_k = [0;0;0];
            q_pred_k = q_k;
            u_k = acc_nogravity;
            acc_k = acc_data;
            % Cuanto tendria que valer P????
            P_pred_k = 100*eye(9);
        elseif print_occgrid == 1
            u_k1 = u_k;
            u_k = acc_nogravity;
            acc_k1 = acc_k;
            acc_k = acc_data;
            if ekf_update == 0
                pos_k1 = pos_pred_k;
                vel_k1 = vel_pred_k;
                q_k1 = q_pred_k;
                P_k1 = P_pred_k;
            end
            % 1) Actualizamos el estado con entradas de la IMU, basados en
            % nuestro estado anterior
            deltat = time_reg - time_prev;
            
            pos_pred_k = pos_k1 + deltat*vel_k1 + ...
                       (deltat*deltat/2)*u_k1;
                   
            vel_pred_k = vel_k1 + deltat*u_k1;
            
            qxx = [0 -q_k1(4) q_k1(3); q_k1(4) 0 -q_k1(2); -q_k1(3) q_k1(2) 0];
            Sigma_q_k1 = q_k1(1,1)*eye(4) + [0 q_k1(2:4,1)';q_k1(2:4,1) qxx];
            q_pred_k = Sigma_q_k1*q_k1;
            
            % 2) Propagamos incertidumbres
            % Definir variables estas
            axx = [0 -acc_k1(3) acc_k1(2); ...
                   acc_k1(3) 0 -acc_k1(1); ...
                   -acc_k1(2) acc_k1(1) 0];
               
            % El error state es de 9 variables
            F_k1 = [eye(3) eye(3)*deltat zeros(3); ...
                    zeros(3) eye(3) -axx*deltat; ...
                    zeros(3) zeros(3) eye(3)];
                
            L_k1 = [zeros(3) zeros(3); ...
                    eye(3) zeros(3); ...
                    zeros(3) eye(3)];

            Q_k1 = deltat*deltat*[sigma_acc*sigma_acc*eye(3) zeros(3); ...
                                  zeros(3) sigma_gyr*sigma_gyr*eye(3)];

            P_pred_k = F_k1*P_k1*F_k1' + L_k1*Q_k1*L_k1';
        end                 
        time_prev = time_reg;

    elseif strcmp(char(bag.MessageList.Topic(i)),'/horizontal_laser_2d') && ...
            start_scans > 0
        
        % guardo el dato de scan actual
        scan_index = scan_index + 1;
        scan_data(scan_index,1) = msgs_cell(i);
        
        Lmin = 10;  % minima cantidad de datos de un contour
        dist_threshold = 0.1;   % distancia minima entre puntos consecutivos
        min_range = scan_data{scan_index,1}.RangeMin;
        max_range = scan_data{scan_index,1}.RangeMax;

        ranges = zeros(length(scan_data{scan_index,1}.Ranges),1);
        % cargo los datos de lidar en un vector
        for j = 1:length(scan_data{scan_index,1}.Ranges)
            % tienen mas de un dato a veces (??)
            scan_data_aux = scan_data{scan_index,1}.Ranges(j).Echoes(1);
            ranges(j,1) = scan_data_aux;   
        end
        % cargo los angulos
        angles = scan_data{scan_index,1}.AngleMin: ...
                 scan_data{scan_index,1}.AngleIncrement:... 
                 scan_data{scan_index,1}.AngleMax;
        
        lidar_data = lidarScan(ranges,angles);
        % extraigo contornos
        lidar_data_reg = contourextraction(lidar_data,Lmin, ...
                                     dist_threshold,min_range,max_range);
        % si ya tuve una medicion anterior del lidar, hago el update
        if print_occgrid == 1
            x = map_poses(map_count,1);
            y = map_poses(map_count,2);
            [data,x,y] = bruteforcesearch(map,lidar_data_reg,x,y,yaw,max_range);
            map_poses_reg = [x y yaw];
            map_poses = [map_poses; map_poses_reg];
            map_count = map_count + 1;
            map_scans{map_count,1} = lidar_data_reg;
            map = buildMap(map_scans,map_poses,50,40);
%             pause(1);
        % caso contrario, creo el mapa suponiendo que el origen es 
        % x = 0 
        % y = 0
        % tita = 0
        else
            % para la primera vez tengo que guardar algun dato de
            % orientacion
            print_occgrid = 1;
            lidar_data_reg = lidarScan(rotateandtranslate2d( ...
                                    lidar_data_reg.Cartesian', ...
                                    yaw,0,0)');
            map_count = map_count + 1;
            map_scans{map_count,1} = lidar_data_reg;
            map_poses = [0 0 0];
            map_poses_reg = [0 0 0];
            map = buildMap(map_scans,map_poses,50,40);
        end
        
        lidar_data_prev = lidar_data_reg;
    end
    i = i + 1;
end

%% Para cada dato del lidar
%{
ranges = zeros(length(scan_data{1,1}.Ranges),1);
for i = 1:length(scan_data{1,1}.Ranges)
    ranges(i,1) = scan_data{1,1}.Ranges(i).Echoes(1);   % tienen mas de un dato a veces (??)
end

angles = scan_data{1,1}.AngleMin:scan_data{1,1}.AngleIncrement:scan_data{1,1}.AngleMax;

lidar_data = lidarScan(ranges,angles);
%}
%% Para cada dato de la IMU
%{
for i = 1:imu_index
    
end
%}
%%
%{
lidar_transposed = rotateandtranslate2d(lidar_data.Cartesian', ...
                                        0,0,0)';
%}

%%

% slamObj = robotics.LidarSLAM(50,10);
% slamObj.LoopClosureThreshold = 360;
% slamObj.LoopClosureSearchRadius = 8;

%{
map_resolution = 50;
map_size = 2;

% for data_state=1:1:length(scans)
    % scan = {lidarScan(points)};
%     if data_state == 1
        pose = [0 0 0];

        map = buildMap(scans(1),pose,map_resolution,map_size);
        map.show;
%} 
%     else
%         map.(scans(data_state));
%     end
% addScan(slamObj,scan);

% slamObj.show;

% pause(0.05)
% end
% plot(points(:,1),points(:,2));

%%

% Primero cargo el mapa inicial con el scan inicial y la posicion en 0,0,0
%{
map_resolution = 50;
map_size = 2;

pose = [0 0 0];

map_scans = scans(1);

test_scan = scans{1};

circ_count = 5;              % cantidad de circunferencias a realizar
X = zeros(length(angles),circ_count);
Y = zeros(length(angles),circ_count);

% Guardo las probabilidades de cada i:jpunto supuesto y matcheado
proba_circ = zeros(length(angles),circ_count);

scanpoints_trasposed = zeros(2,length(angles));
scans_index = 2;
i = 1;
k = 1;

% genero las circunferencias para poder hacer el algoritmo de fuerza bruta
figure(2);

for i=1:1:circ_count
    r = i / map_resolution;
    X(:,i) = r * cos(angles);
    Y(:,i) = r * sin(angles);
    
    for j=1:1:length(angles)
        actual_range = scans{scans_index}.Ranges(j);
        if actual_range > min_range && actual_range < max_range
            % Rigid-Body transformation
            scanpoints_trasposed(:,k) = rotateandtranslate2d(scans{scans_index}.Cartesian(j,:)', ...
                                                    X(j,i),Y(j,i),0);
            k = k + 1;
        end
    end
    
    hold on
    plot(scanpoints_trasposed(1,1:k),scanpoints_trasposed(2,1:k));
    k = 1;
end
%}


%% de Gao2018

%{
d = 0.03;
e = 0.03;
Snum = 6;
Pmin = 10;
Lmin = 0.6;
num_scans = 10;

S = seed(scans{num_scans},max_range,min_range,e,d,Snum,Pmin,Lmin);

out = overlapregion(scans{num_scans},S);

figure;
for i=1:length(out)
    m = out(i).m;
    b = out(i).b;
    x = out(i).x;
    plot(x,m*x+b);
    hold on
    plot(out(i).LaserPoints(:,1),out(i).LaserPoints(:,2));
end
xlim([-map_size map_size]);
ylim([-map_size map_size]);
%}

%% Mas croto
%{
dist_threshold = 0.1;
Lmin = 5;
num_scans = 2;
data = scans{num_scans}.Cartesian;
ranges = scans{num_scans}.Ranges;

i = 2;
contours = {};
a = 1;
data_2 = [];
while i <= length(data)
    j = i;
    while point2pointdist(data(i,:),data(i-1,:)) <= dist_threshold
        i = i + 1;
    end
    if length(j:i) >= Lmin
        aux = lidarScan(data(j:i-1,:));
        contours{a,1} = aux;
        data_2 = [data_2;data(j:i-1,:)];
        a = a + 1;
    end
    if ranges(i) > max_range || ranges(i) < min_range
        i = i + 1;
    end
    i = i + 1;
end
%}
% figure;
% contour_map = buildMap({lidarScan(data_2)},pose,map_resolution,map_size);
% contour_map = buildMap({contours{1,1}},pose,map_resolution,map_size);
% contour_map.show;

%% Brute force search
%{
dist_threshold = 0.1;
Lmin = 5;
num_scans = 3;
data = scans{num_scans}.Cartesian;
ranges = scans{num_scans}.Ranges;

tita = pi/32;
tx = 0;
ty = 0;

data_3 = [];
i = 1;
while i <= length(data)
    if ranges(i) < max_range && ranges(i) > min_range
        data_3 = [data_3; data(i,:)];
    end
    i = i + 1;
end

figure;
plot(data_2(:,1),data_2(:,2));
hold on
plot(data_3(:,1),data_3(:,2));

% contours_transformed = rotateandtranslate2d(data_2',tita,tx,ty)';
%}