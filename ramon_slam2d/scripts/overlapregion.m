% line_segments - vector of line segments
% m             - vector of line slopes
% b             - vector of line initial values

function out = overlapregion(scans,line_segments)
    Nl = length(line_segments);
    out = LineSegmentClass;
    laser_points = scans.Cartesian;
    angles = scans.Angles;
    i = 1;
    while i < Nl-1
        j = i+1;
        if i == 1
            % Endpoint index of Line i (m1,n1)
            m1 = line_segments(i).Pb; 
            n1 = line_segments(i).Pf;
            m_i = line_segments(i).m;
            b_i = line_segments(i).b;
        else
            m1 = out(i).Pb; 
            n1 = out(i).Pf;
            m_i = out(i).m;
            b_i = out(i).b;
        end            
        % Endpoint index of line j (m2,n2)
        m2 = line_segments(j).Pb; 
        n2 = line_segments(j).Pf;
        m_j = line_segments(j).m;
        b_j = line_segments(j).b;
        
        if m2 <= n1
            for k = m2:n1
                di = point2linedist(laser_points(k,:),m_i,b_i);
                dj = point2linedist(laser_points(k,:),m_j,b_j);
                if di > dj
                    break
                end
            end
            n1 = k - 1;
            m2 = k;
        end
        
        out(i).LaserPoints = [laser_points(m1:n1,1) laser_points(m1:n1,2)];
        [ml,bl] = linesegment(out(i).LaserPoints);        
        tita = angles(m1);
        % Punto predecido k
        xp_1 = (bl*cos(tita)) / (sin(tita) - ml*cos(tita));

        tita = angles(n1);
        xp_2 = (bl*cos(tita)) / (sin(tita) - ml*cos(tita));

        out(i).m = ml;
        out(i).b = bl;
        out(i).x = [xp_1 ; xp_2];
        out(i).Pb = m1;
        out(i).Pf = n1;


        out(i+1).LaserPoints = [laser_points(m2:n2,1) laser_points(m2:n2,2)];
        [ml,bl] = linesegment(out(i+1).LaserPoints);
        
        tita = angles(m2);
        % Punto predecido k
        xp_1 = (bl*cos(tita)) / (sin(tita) - ml*cos(tita));

        tita = angles(n2);
        xp_2 = (bl*cos(tita)) / (sin(tita) - ml*cos(tita));

        out(i+1).m = ml;
        out(i+1).b = bl;
        out(i+1).x = [xp_1 ; xp_2];
        out(i+1).Pb = m2;
        out(i+1).Pf = n2;   
        
        i = i + 1;
    end
end