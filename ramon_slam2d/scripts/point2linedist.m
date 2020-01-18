% point [x y]
% m     pendiente de la recta
% b     ordenada al origen de la recta
function d = point2linedist(point,m,b)
    d = abs((point(2) - m*point(1) - b) / sqrt(m*m + 1));
end