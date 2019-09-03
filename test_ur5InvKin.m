%% initialiazation
rosshutdown;
ur5 = ur5_interface();
ur5.init_gripper;

%% off-set
g_baseK_S = [ROTZ(-pi/2) [0 0 0.0892]'; 0 0 0 1];  
baseKFrame = tf_frame('S','base_K',eye(4));
baseKFrame.move_frame('S',inv(g_baseK_S));
g_T_toolK = [ROTX(-pi/2)*ROTY(pi/2) [0 0 0]'; 0 0 0 1];   
toolKFrame = tf_frame('T','tool_K',eye(4));
toolKFrame.move_frame('T',g_T_toolK);

%% learning pick checkpoint
%joints_pick = [pi/8; -pi/4; pi/2; -3*pi/4; -pi/2; 0];
joints_pick = ur5.get_current_joints;
% g_pick = ur5FwdKin(joints_pick);
g_pick = ur5FwdKin(joints_pick);
%% learning place checkponit
%joints_place = [-pi/8; -pi/4; pi/2; -3*pi/4; -pi/2; 0];
joints_place = ur5.get_current_joints;
% g_place = ur5FwdKin(joints_place);
g_place = ur5FwdKin(joints_place);
%% calculating pre point
height = 0.1; % offset of pre point
g_pre_pick = g_pick + [zeros(3) [0 0 height]';0 0 0 0];
g_pre_place = g_place +[zeros(3) [0 0 height]';0 0 0 0];
%% creating target frame
Frame1 = tf_frame('base','place',g_place);
Frame2 = tf_frame('base','pick',g_pick);
Frame3 = tf_frame('base','pick_pre',g_pre_pick);
Frame4 = tf_frame('base','place_pre',g_pre_place);
%% inverse kinematic method pick
thetas_pre_pick = ur5InvKin(g_pre_pick);
thetas_pick = ur5InvKin(g_pick);
ur5.move_joints(ur5.home, 5);
waitforbuttonpress;
ur5.move_joints(thetas_pre_pick(:,6), 5);
waitforbuttonpress;
ur5.open_gripper;
waitforbuttonpress;
ur5.move_joints(thetas_pick(:,6), 5);
waitforbuttonpress;
ur5.close_gripper;
waitforbuttonpress;
ur5.move_joints(thetas_pre_pick(:,6), 5);
waitforbuttonpress;
thetas_pre_place = ur5InvKin(g_pre_place);
thetas_place = ur5InvKin(g_place);
ur5.move_joints(thetas_pre_place(:,6), 5);
waitforbuttonpress;
ur5.move_joints(thetas_place(:,6), 5);
waitforbuttonpress;
ur5.open_gripper;
waitforbuttonpress;
ur5.move_joints(thetas_pre_place(:,6), 5);
waitforbuttonpress;
ur5.move_joints(ur5.home, 5);
waitforbuttonpress;
ur5.close_gripper;
%% RRT control method
ur5.move_joints(thetas_pick(:,6)+[0 pi/8 pi/8 0 0 0]', 5);
waitforbuttonpress;
ur5RRcontrol(g_pre_pick, 1.7, ur5);
waitforbuttonpress;
ur5.open_gripper;
waitforbuttonpress;
ur5RRcontrol(g_pick, 1.7, ur5);
waitforbuttonpress;
ur5.close_gripper;
waitforbuttonpress;
ur5RRcontrol(g_pre_pick, 1.7, ur5);
waitforbuttonpress;
ur5RRcontrol(g_pre_place, 1.7, ur5);
waitforbuttonpress;
ur5RRcontrol(g_place, 1.7, ur5);
waitforbuttonpress;
ur5.open_gripper;
waitforbuttonpress;
ur5RRcontrol(g_pre_place, 1.7, ur5);
waitforbuttonpress;
ur5.close_gripper;
ur5.move_joints(ur5.home, 5);
%% transpose jacobian method
ur5.move_joints(thetas_pick(:,6)+[0 pi/8 pi/8 0 0 0]', 5);
waitforbuttonpress;
ur5JRRcontrol(g_pre_pick, 0.6, ur5, 0.03, 0.03);
waitforbuttonpress;
ur5.open_gripper;
waitforbuttonpress;
ur5JRRcontrol(g_pick, 0.8, ur5, 0.01, 0.01);
waitforbuttonpress;
ur5.close_gripper;
waitforbuttonpress;
ur5JRRcontrol(g_pre_pick, 0.8, ur5, 0.03, 0.03);
waitforbuttonpress;
ur5JRRcontrol(g_pre_place + [zeros(3) [0 0 0.1]';0 0 0 0], 1, ur5, 0.03, 0.03);
waitforbuttonpress;
ur5JRRcontrol(g_place , 0.8, ur5, 0.01, 0.01);
waitforbuttonpress;
ur5.open_gripper;
waitforbuttonpress;
ur5JRRcontrol(g_pre_place, 0.8, ur5, 0.03, 0.03);
waitforbuttonpress;
ur5.close_gripper;
ur5.move_joints(ur5.home, 5);




