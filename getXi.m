function xi = getXi( g )
xi_hat = logm(g);
w_hat = xi_hat(1 : 3, 1 : 3);
v = xi_hat(1 : 3, 4);
w2 = w_hat(1, 3);
w3 = w_hat(2, 1);
w1 = w_hat(3 ,2);
xi = [v; w1; w2; w3];
end