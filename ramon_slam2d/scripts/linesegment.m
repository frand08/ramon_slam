function [m,b] = linesegment(laser_points)
    x_mean = mean(laser_points(:,1));
    y_mean = mean(laser_points(:,2));
    xx_mean = mean(laser_points(:,1).*laser_points(:,1));
    yy_mean = mean(laser_points(:,2).*laser_points(:,2));
    
    xy_mean = mean(laser_points(:,1).*laser_points(:,2));
    
    sig_xy = xy_mean - x_mean*y_mean;
    sig_y = sqrt(yy_mean - y_mean*y_mean);
    sig_x = sqrt(xx_mean - x_mean*x_mean);
    
    R = (sig_x*sig_x - sig_y*sig_y) / sig_xy;
    
    m_1 = (-R + sqrt(R*R + 4)) / 2;
    m_2 = (-R - sqrt(R*R + 4)) / 2;
    
    if m_1*sig_xy > 0 && m_2*sig_xy < 0
        m = m_1;
    elseif m_1*sig_xy < 0 && m_2*sig_xy > 0
        m = m_2;
    else
        m = 0;          % Errorrrrrr
    end
    
    b = y_mean - m*x_mean; 
%     plot(laser_points(:,1),laser_points(:,2),'r'); M1 = "LIDAR";
%     hold on
%     plot(laser_points(:,1),m*laser_points(:,1)+b,'b'); M2 = "Curva";
%     legend([M1; M2]);
end