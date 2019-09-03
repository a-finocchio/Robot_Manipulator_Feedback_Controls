function M = XF(vector)
x = vector(1,:);
y = vector(2,:);
z = vector(3,:);
theta1 = vector(4,:);
theta2 = vector(5,:);
theta3 = vector(6,:);
euler = EULERXYZ([theta1;theta2;theta3]);
translation = [x ; y; z];
M = [euler translation; 0 0 0 1];

