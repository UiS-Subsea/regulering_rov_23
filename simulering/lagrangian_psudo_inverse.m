motor_calculations;

u = [30 30 30 30 0 0 0 0];
tau = B*(u');
tau(5) = 0;
tau(6) = 0;

W = diag([1,1,1,1,1,1,1,1]);
Bt = transpose(B); Wi = inv(W);
Bw = Wi*Bt*inv(B*Wi*Bt);
u_opt_newton = (Bw*tau)'

% hvis W = 1...1, så er Bw = pinv(B)
Bw = pinv(B)
u_opt_pseudo = (Bw*tau)'