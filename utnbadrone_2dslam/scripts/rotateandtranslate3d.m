%%A utility function to do the transformation
% P: Rigid body to transform
% rotdata: translation of pos=[x;y;z] + rotation q=[qw;qx;qy;qz]
function P = rotateandtranslate3d(P,q,pos)
    %First do rotation
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    x = pos(1);
    y = pos(2);
    z = pos(3);
    s = 1/abs(qw*qw + qx*qx + qy*qy + qz*qz);
    R = [1-2*s*(qy*qy+qz*qz) 2*s*(qx*qy-qz*qw)   2*s*(qx*qz+qy*qw); ...
         2*s*(qx*qy+qz*qw)   1-2*s*(qx*qx+qz*qz) 2*s*(qy*qz-qx*qw); ...
         2*s*(qx*qz-qy*qw)   2*s*(qy*qz+qx*qw)   1-2*s*(qx*qx+qy*qy)];
    P = R * P;
    %Then translation 
    P = bsxfun(@plus,P,[x;y;z]);
end