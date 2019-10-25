function d = point2pointdist(point_a,point_b)
    d = sqrt((point_a(1)-point_b(1))*(point_a(1)-point_b(1)) + ...
             (point_a(2)-point_b(2))*(point_a(2)-point_b(2)));
end