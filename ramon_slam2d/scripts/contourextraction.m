function [lidar_out] = contourextraction(lidar_data,Lmin, ...
                                           dist_threshold, ...
                                           min_range,max_range)
    data = lidar_data.Cartesian;
    ranges = lidar_data.Ranges;
    ranges_out = lidar_data.Ranges;
    
    i = 2;
    while i <= length(data)
        j = i;
        while point2pointdist(data(i,:),data(i-1,:)) < dist_threshold && ...
                ranges(i-1) < max_range && ranges(i-1) > min_range
            if i == length(data)
                break
            else
                i = i + 1;
            end
        end
        if length(j:i) < Lmin
            ranges_out(j:i-1) = max_range;
        end
        if ranges(i) >= max_range || ranges(i) <= min_range || j - i <= 0
            ranges_out(i) = max_range;
%             i = i + 1;
        end
        i = i + 1;
    end
    lidar_out = lidarScan(ranges_out,lidar_data.Angles);
end