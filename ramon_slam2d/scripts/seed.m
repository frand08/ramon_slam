% De Gao2018
function S = seed(scans,max_range,min_range,e,d,Snum,Pmin,Lmin)
    laser_points = scans.Cartesian;
    angles = scans.Angles;
    Np = length(laser_points(:,1));

    S = LineSegmentClass;
    S2 = {};
    
    flag = 1;
    cant = 0;
    i = 1;
    xp = zeros(Snum,1);
    yp = zeros(Snum,1);
    
    while i < (Np-Pmin)
        j = i + Snum;
        [m,b] = linesegment([laser_points(i:j,1) laser_points(i:j,2)]);
        for k=i:1:j
            if scans.Ranges(k) > max_range || scans.Ranges(k) < min_range
                flag = 0;
                i = k;
                break;
            end
            tita = angles(k);
            % Punto predecido k
            yp(k-i+1) = (m*b*cos(tita) / (sin(tita) - m*cos(tita))) + b;
            xp(k-i+1) = (b*cos(tita)) / (sin(tita) - m*cos(tita));
            
            % Distancia del punto k al punto predecido k
            d1 = point2pointdist(laser_points(k,:),[xp(k-i+1) yp(k-i+1)]);
 
            % Distancia del punto k al seed segment (i,j)
            err = point2linedist(laser_points(k,:),m,b);
                   
            if err > e || d1 > d
                flag = 0;
                break;
            end
        end
    
        if flag == 1
            [Pb,Pf,ml,bl] = rgrowing(scans,max_range,min_range,...
                                     Pmin,Lmin,e,d,j,i);
            if Pf > 0
                tita = angles(Pb);
                % Punto predecido k
                xp_1 = (bl*cos(tita)) / (sin(tita) - ml*cos(tita));

                tita = angles(Pf);
                xp_2 = (bl*cos(tita)) / (sin(tita) - ml*cos(tita));
                cant = cant + 1;
                S(cant).x = [xp_1; xp_2];
                S(cant).m = ml;
                S(cant).b = bl;
                S(cant).Pb = Pb;
                S(cant).Pf = Pf;
                S(cant).LaserPoints = [laser_points(Pb:Pf,1) 
                                       laser_points(Pb:Pf,2)];
                i = Pf;
            end
        else
            flag = 1;
        end
        i = i + 1;
    end
end