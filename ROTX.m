%the following is the function code for ROTX
function A = ROTX(rolls)

A = [1 0 0;
     0 cos(rolls) -sin(rolls)
     0 sin(rolls) cos(rolls)];
end
