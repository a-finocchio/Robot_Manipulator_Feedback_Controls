function finalerr = ur5JRRcontrol(gdesired, K, ur5, th_v, th_w)
q0 = ur5.get_current_joints();
q = q0;
% th_v = 0.01;
% th_w = 0.01;
step = 0.5;
step_shuchu = 0.5;
count = 0;
while(1)
if abs(det(ur5BodyJacobian( q ))) < 0.0001
    fprintf('singularity')
    finalerr = -1;
    break
end
count = count + 1;
step_shuchu = step_shuchu + 0.001 * count
xi = getXi(gdesired \  ur5FwdKin(q) );
q = q - K * step * transpose(ur5BodyJacobian( q )) * xi;
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
    