%% Leo el bag
clc;close all;clear;
% bag = rosbag('_2019-10-14-10-38-58_pieza_fondo_1hz.bag');
bag = rosbag('_2019-10-15-15-19-10_pieza_fondo_1hz.bag');
msgs_cell = readMessages(bag);      %% Obtengo todos los mensajes
cant_msgs = length(msgs_cell);

%% Separo scans del resto
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

%%

% slamObj = robotics.LidarSLAM(50,10);
% slamObj.LoopClosureThreshold = 360;
% slamObj.LoopClosureSearchRadius = 8;

map_resolution = 50;
map_size = 2;

% for data_state=1:1:length(scans)
    % scan = {lidarScan(points)};
%     if data_state == 1
        pose = [0 0 0];

        map = buildMap(scans(1),pose,map_resolution,map_size);
        map.show;
        
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
%{
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

% %{
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
% %}

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