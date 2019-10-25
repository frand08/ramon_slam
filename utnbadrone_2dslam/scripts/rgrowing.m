% De Gao2018

% Inputs:
% scans  - lidarScan data
% max_range
% min_range
% S      - seed segment
% Pmin   - minimum number of laser points contained in an extracted line
%          segment
% Lmin   - minimum length requirement of an extracted line segment
% e      - point to line distance threshold
% d      - point to point distance threshold
% i      - counter from seed function

% Outputs:
% points - X axis points returned from line
% ml     - slope of the line extracted
% bl     - initial value of the line extracted

function [points,ml,bl,i] = rgrowing(scans,max_range,min_range,S,Pmin,Lmin,e,d,j,i)
    laser_points = scans.Cartesian;
    angles = scans.Angles;
    Np = length(laser_points(:,1));

    points = S(:,1);
    
    [ml,bl] = linesegment([laser_points(i:j,1) laser_points(i:j,2)]);

    Pf = j + 1;
    Pb = i - 1;
            
    tita = angles(Pf);
    % Punto predecido k
    yp = (ml*bl*cos(tita) / (sin(tita) - ml*cos(tita))) + bl;
    xp = (bl*cos(tita)) / (sin(tita) - ml*cos(tita));    
    
    if Pf <= Np && scans.Ranges(Pf) < max_range && scans.Ranges(Pf) > min_range
        while point2linedist(laser_points(Pf,:),ml,bl) < e && ...
              point2pointdist(laser_points(Pf,:),[xp yp]) < e

            [ml,bl] = linesegment([laser_points(i:Pf,1) laser_points(i:Pf,2)]);
            Pf = Pf + 1;
            if Pf > Np || scans.Ranges(Pf) > max_range || ...
               scans.Ranges(Pf) < min_range
                break
            else                
                tita = angles(Pf);
                % Punto predecido k
                yp = (ml*bl*cos(tita) / (sin(tita) - ml*cos(tita))) + bl;
                xp = (bl*cos(tita)) / (sin(tita) - ml*cos(tita)); 
            end
        end
    end
    Pf = Pf - 1;
    

    tita = angles(Pb);
    % Punto predecido k
    yp = (ml*bl*cos(tita) / (sin(tita) - ml*cos(tita))) + bl;
    xp = (bl*cos(tita)) / (sin(tita) - ml*cos(tita)); 
    
    if Pb >= 1 && scans.Ranges(Pb) < max_range && scans.Ranges(Pb) > min_range
        while point2linedist(laser_points(Pf,:),ml,bl) < e && ...
              point2pointdist(laser_points(Pb,:),[xp yp]) < e
            [ml,bl] = linesegment([laser_points(Pb:Pf,1) laser_points(Pb:Pf,2)]);
            Pb = Pb - 1;
            if Pb < 1 || scans.Ranges(Pb) > max_range || ...
               scans.Ranges(Pb) < min_range
                break
            else                
                tita = angles(Pb);
                % Punto predecido k
                yp = (ml*bl*cos(tita) / (sin(tita) - ml*cos(tita))) + bl;
                xp = (bl*cos(tita)) / (sin(tita) - ml*cos(tita)); 
            end
        end
    end    
    Pb = Pb + 1;
    
    % obtain Ll, Pl from line(Pb, Pf)
    
    % Real length of an extracted line segment
    Ll = sqrt(laser_points(Pb,1)*laser_points(Pb,1) + ...
              laser_points(Pf,1)*laser_points(Pf,1));
    
    % Number of laser points contained in the line segment
    Pl = length(Pb:Pf); 
    if Ll >= Lmin && Pl >= Pmin
        % deberia devolver ml, bl y los puntos creo (i digamos)
        points = double(laser_points(Pb:Pf,1));
        i = Pf;
    end
end