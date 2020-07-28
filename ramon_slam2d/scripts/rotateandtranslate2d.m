%%A utility function to do the transformation
% P: Rigid body to transform
% rotdata: [translation of (x,y) + rotation tita]
function P = rotateandtranslate2d(P,tita,x,y)
    %First do rotation
    P = [cos(tita) -sin(tita); sin(tita) cos(tita)] * P;
    %Then translation 
    P = bsxfun(@plus,P,[x;y]);
end