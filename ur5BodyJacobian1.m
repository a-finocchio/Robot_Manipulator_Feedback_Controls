function J_b = ur5BodyJacobian1(joints)
% input: joints is 6*1 vector where joints (i) correspond to joint i in
% gazebo setting.
% output: J_b is body Jacobian
l_1=0.0892;    % m
l_2=0.425;
l_3=0.39225;
l_4=0.09465;
l_5=0.10915;
l_6=0.0823;

offset=[pi/2;pi/2;0;pi/2;0;0];
joints=joints+offset;

gst_0=[1,0,0,l_5+l_6;
       0,1,0,0;
       0,0,1,l_1+l_2++l_3+l_4;
       0,0,0,1];

w_1=[0;0;1];   
w_2=[1;0;0];
w_3=[1;0;0];
w_4=[1;0;0];
w_5=[0;0;1];
w_6=[1;0;0];
q_1=[0;0;0];
q_2=[0;0;l_1];
q_3=[0;0;l_1+l_2];
q_4=[0;0;l_1+l_2+l_3];
q_5=[l_5;0;0];
q_6=[0;0;l_1+l_2+l_3+l_4];
w_hat_1=[0,-1,0;1,0,0;0,0,0];
w_hat_5=w_hat_1;
w_hat_2=[0,0,0;0,0,-1;0,1,0];
w_hat_3=w_hat_2;
w_hat_4=w_hat_2;
w_hat_6=w_hat_2;
v_1=cross(-w_1,q_1);
v_2=cross(-w_2,q_2);
v_3=cross(-w_3,q_3);
v_4=cross(-w_4,q_4);
v_5=cross(-w_5,q_5);
v_6=cross(-w_6,q_6);

twist_1=[v_1;w_1];
twist_2=[v_2;w_2];
twist_3=[v_3;w_3];
twist_4=[v_4;w_4];
twist_5=[v_5;w_5];
twist_6=[v_6;w_6];

twist_hat_1=[w_hat_1,v_1;zeros(1,4)];
twist_hat_2=[w_hat_2,v_2;zeros(1,4)];
twist_hat_3=[w_hat_3,v_3;zeros(1,4)];
twist_hat_4=[w_hat_4,v_4;zeros(1,4)];
twist_hat_5=[w_hat_5,v_5;zeros(1,4)];
twist_hat_6=[w_hat_6,v_6;zeros(1,4)];

g_st=expm(twist_hat_1.*joints(1))*expm(twist_hat_2.*joints(2))*expm(twist_hat_3.*joints(3))*expm(twist_hat_4.*joints(4))*expm(twist_hat_5.*joints(5))*expm(twist_hat_6.*joints(6))*gst_0;

% twist_2' calculation
e2=expm(twist_hat_1.*joints(1));
e2_p=e2(1:3,4);
e2_p_hat=[0,-e2_p(3),e2_p(2);
          e2_p(3),0,-e2_p(1);
          -e2_p(2),e2_p(1),0];
e2_R=e2(1:3,1:3);
e2_Ad=[e2_R,e2_p_hat*e2_R;zeros(3,3),e2_R];
twist_2_prime=e2_Ad*twist_2;

% twist_3' calculation
e3=expm(twist_hat_1.*joints(1))*expm(twist_hat_2.*joints(2));
e3_p=e3(1:3,4);
e3_p_hat=[0,-e3_p(3),e3_p(2);
          e3_p(3),0,-e3_p(1);
          -e3_p(2),e3_p(1),0];
e3_R=e3(1:3,1:3);
e3_Ad=[e3_R,e3_p_hat*e3_R;zeros(3,3),e3_R];
twist_3_prime=e3_Ad*twist_3;

% twist_4' calculation
e4=expm(twist_hat_1.*joints(1))*expm(twist_hat_2.*joints(2))*expm(twist_hat_3.*joints(3));
e4_p=e4(1:3,4);
e4_p_hat=[0,-e4_p(3),e4_p(2);
          e4_p(3),0,-e4_p(1);
          -e4_p(2),e4_p(1),0];
e4_R=e4(1:3,1:3);
e4_Ad=[e4_R,e4_p_hat*e4_R;zeros(3,3),e4_R];
twist_4_prime=e4_Ad*twist_4;

% twist_5' calculation
e5=expm(twist_hat_1.*joints(1))*expm(twist_hat_2.*joints(2))*expm(twist_hat_3.*joints(3))*expm(twist_hat_4.*joints(4));
e5_p=e5(1:3,4);
e5_p_hat=[0,-e5_p(3),e5_p(2);
          e5_p(3),0,-e5_p(1);
          -e5_p(2),e5_p(1),0];
e5_R=e5(1:3,1:3);
e5_Ad=[e5_R,e5_p_hat*e5_R;zeros(3,3),e5_R];
twist_5_prime=e5_Ad*twist_5;

% twist_6' calculation
e6=expm(twist_hat_1.*joints(1))*expm(twist_hat_2.*joints(2))*expm(twist_hat_3.*joints(3))*expm(twist_hat_4.*joints(4))*expm(twist_hat_5.*joints(5));
e6_p=e6(1:3,4);
e6_p_hat=[0,-e6_p(3),e6_p(2);
          e6_p(3),0,-e6_p(1);
          -e6_p(2),e6_p(1),0];
e6_R=e6(1:3,1:3);
e6_Ad=[e6_R,e6_p_hat*e6_R;zeros(3,3),e6_R];
twist_6_prime=e6_Ad*twist_6;

J_s=[twist_1,twist_2_prime,twist_3_prime,twist_4_prime,twist_5_prime,twist_6_prime];

g_st_inv=inv(g_st);
gp=g_st_inv(1:3,4);                     % p part of g inverse 
gp_hat=[0,-gp(3),gp(2);
          gp(3),0,-gp(1);
          -gp(2),gp(1),0];
g_R=g_st_inv(1:3,1:3);                  % R part of g inverse
g_st_inv_Ad=[g_R,gp_hat*g_R;zeros(3,3),g_R];

J_b=g_st_inv_Ad*J_s;

end