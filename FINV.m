function M = FINV(N)
euler = N(1:3,1:3);
translation = N(1:3,4);
post_translation = -euler' * translation
M = [euler' post_translation;0 0 0 1];