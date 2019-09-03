%% initialiazation
rosshutdown;
ur5 = ur5_interface();
ur5.init_gripper;

%% off-set
% g_baseK_S = [ROTZ(-pi/2) [0 0 0.0892]'; 0 0 0 1];  
% baseKFrame = tf_frame('S','base_K',eye(4));
% baseKFrame.move_frame('S',inv(g_baseK_S));
% g_T_toolK = [ROTX(-pi/2)*ROTY(pi/2) [0 0 0]'; 0 0 0 1];   
% toolKFrame = tf_frame('T','tool_K',eye(4));
% toolKFrame.move_frame('T',g_T_toolK);

%% learning start position
safe_joints_pick = ur5.get_current_joints;
g_pick = ur5FwdKin(safe_joints_pick);
v_safe= g_pick(1:3,4);
hs=g_pick(3,4);
g_pickup = ur5FwdKin(safe_joints_pick)+[0 0 0 0; 0 0 0 0; 0 0 0 0.1; 0 0 0 0;];
j_pickup = ur5InvKin(g_pickup);

%% calibration
%calculations
g_cali1=[eye(3),v_safe+[-0.1; 0; 0;]; 0 0 0 1;];
g_cali1up=[eye(3),v_safe+[-0.1; 0; 0.1;]; 0 0 0 1;];
g_cali2=[eye(3),v_safe+[0; 0.1; 0;]; 0 0 0 1;];
g_cali2up=[eye(3),v_safe+[0; 0.1; 0.1;]; 0 0 0 1;];
g_cali3=[eye(3),v_safe+[0.1; 0.1; 0;]; 0 0 0 1;];
g_cali3up=[eye(3),v_safe+[0; 0.1; 0.1;]; 0 0 0 1;];

j_cali1=ur5InvKin(g_cali1);
j_cali1up=ur5InvKin(g_cali1up);
j_cali2=ur5InvKin(g_cali2);
j_cali2up=ur5InvKin(g_cali2up);
j_cali3=ur5InvKin(g_cali3);
j_cali3up=ur5InvKin(g_cali3up);

%calibration move
ur5.move_joints(j_pickup(:,6), 5);
waitforbuttonpress;

ur5.move_joints(j_cali1up(:,6), 5);
waitforbuttonpress;
ur5.move_joints(j_cali1(:,6), 5);
waitforbuttonpress;
ur5.move_joints(j_cali1up(:,6), 5);
waitforbuttonpress;

ur5.move_joints(j_cali2up(:,6), 5);
waitforbuttonpress;
ur5.move_joints(j_cali2(:,6), 5);
waitforbuttonpress;
ur5.move_joints(j_cali2up(:,6), 5);
waitforbuttonpress;

ur5.move_joints(j_cali3up(:,6), 5);
waitforbuttonpress;
ur5.move_joints(j_cali3(:,6), 5);
waitforbuttonpress;
ur5.move_joints(j_cali3up(:,6), 5);
waitforbuttonpress;

ur5.move_joints(j_pickup(:,6), 5);
waitforbuttonpress;