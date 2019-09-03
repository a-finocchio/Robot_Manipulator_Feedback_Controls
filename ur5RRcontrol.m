function finalerr = ur5RRcontrol(gdesired, K, ur5)
q0 = ur5.get_current_joints();
q = q0;
th_v = 0.01;
th_w = 0.01;
step = 0.4;
count = 0;
while(1)
xi = getXi(gdesired \ ur5FwdKin(q) );
if abs(det(ur5BodyJacobian( q ))) < 0.0001
    fprintf('singularity')
    finalerr = -1;
    break
end
count = count + 1;
step = step + 0.001 * count ;
q = q - K * step * inv(ur5BodyJacobian( q )) * xi;
% pause(1);
ur5.move_joints(q,10);
pause(5);
v = xi(1 : 3);
w = xi(4 : 6);
if (norm(w) < th_w) && (norm(v) < th_v)
    fprintf('positional error:')
    finalerr = norm(v)
    break
end
end
    