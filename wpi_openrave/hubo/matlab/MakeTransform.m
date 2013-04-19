function tm = MakeTransform(rot,trans)

if size(rot,1) == 9 && size(rot,2) == 1
    tm = reshape(rot,3,3);

elseif size(rot,1) == 3 && size(rot,2) == 3
    tm = rot;
else
    error('rotation improperly specified');
end

if size(trans,1) == 3 && size(trans,2) == 1
    tm = [tm,trans];
elseif size(trans,1) == 1 && size(trans,2) == 3
    tm = [tm,trans'];
else
    error('translation improperly specified');
end

tm = [tm;[ 0 0 0 1]];