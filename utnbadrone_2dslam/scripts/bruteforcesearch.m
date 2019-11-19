function [lidar_data_out,x,y] = bruteforcesearch(map,lidar_data,x,y,tita,max_range)
    % primero, roto el dato del lidar y calculo su probabilidad respecto al
    % mapa
    lidar_data_rotated = lidarScan(rotateandtranslate2d( ...
                                       lidar_data, ...
                                       tita,x,y)');
    occupancy_prob_posta = 0;
    for i=1:lidar_data_rotated.Count
        if lidar_data_rotated.Ranges(i) < max_range*0.9
            occupancy_prob_posta = occupancy_prob_posta + ...
            (1-getOccupancy(map,lidar_data_rotated.Cartesian(i,:)))^2;
        end
    end
    lidar_data_out = lidar_data_rotated;
    % Ahora clavo traslaciones chicas a ver que onda    
    circ_count = 10;
    r_count = 9;
    r = 1/100;
    tx = zeros(circ_count*r_count,1);
    ty = zeros(circ_count*r_count,1);
    theta = linspace(0,2*pi,circ_count);
    for h=1:r_count
        for j=1:1:circ_count
            tx(j+(h-1)*circ_count) = x + r*h*cos(theta(j));
            ty(j+(h-1)*circ_count) = y + r*h*sin(theta(j));
            lidar_data_transformed = lidarScan(rotateandtranslate2d( ...
                                               lidar_data_rotated.Cartesian', ...
                                               0, ...
                                               x+tx(j+(h-1)*circ_count), ...
                                               y+ty(j+(h-1)*circ_count))');
            occupancy_prob = 0;
            for i=1:lidar_data_transformed.Count
                if lidar_data_transformed.Ranges(i) < max_range*0.9
                    occupancy_prob = occupancy_prob + ...
                    (1-getOccupancy(map,lidar_data_transformed.Cartesian(i,:)))^2;
                end
            end
            if occupancy_prob < occupancy_prob_posta
                x = x + tx(j+(h-1)*circ_count);
                y = y + ty(j+(h-1)*circ_count);
                occupancy_prob_posta = occupancy_prob;
                lidar_data_out = lidar_data_transformed;
            end
        end
    end
%     map.show;
end