motor_calculations;
disp("KKT solver started");

syms Fx Fy Fz Tx Ty Tz real
tau = [Fx Fy Fz Tx Ty Tz];
syms U1 U2 U3 U4 U5 U6 U7 U8 real
U = [U1 U2 U3 U4 U5 U6 U7 U8];
syms l1 l2 l3 l4 l5 l6 l7 l8 l9 l10 l11 l12 l13 l14 l15 l16 real
mu = [l1 l2 l3 l4 l5 l6 l7 l8 l9 l10 l11 l12 l13 l14 l15 l16]';

Kf = 1e4;
Ke = 1;

Fx = 1;
Fy = 2;
Fz = 3;
Tx = 4;
Ty = 5;
Tz = 6;
f = Kf * ((Fx - sum(B(1, :) .* U))^2 + (Fy - sum(B(2, :) .* U))^2 + (Fz - sum(B(3, :) .* U))^2 + ...
           (Tx - sum(B(4, :) .* U))^2 + (Ty - sum(B(5, :) .* U))^2 + (Tz - sum(B(6, :) .* U))^2) + ...
           Ke * sum(U.^2);

U = U';

% kkt conditions matrix form
umax = 40; % N
umin = -31.4; % N

g = sym([]);
%dg = sym([]);
for i = 1:8
    g = [g; U(i) - umax; umin - U(i)]; % Formulating constraints
end

% Lagrangian and partial derivative
lg = f + mu'*g;
lgd = jacobian(lg,[U]);

% system of equations
equations = [
     lgd'==0;
     g <= 0;
     mu >= 0;
     mu'*g == 0;
];
disp("solving...");
solve(equations)

return;

%% Print in format we can use on the microcontroller for the gradient function
sympref('FloatingPointOutput',true);
df = jacobian(f,U);
hf = hessian(f,U);
df = collect(df);

variables = [tau, U'];

coeffmap = zeros(numel(df), numel(variables));

% extract coeffs
for i = 1:numel(df)
    dfi = df(i);
    for j = 1:numel(variables)
        variable = variables(j);
        c = coeffs(dfi, variable);
        if numel(c) ~= 2
            continue;
        end

        coeffmap(i,j) = c(2);
    end
end


% Corrected format string
fmt = 'static matrix_t gradient_map[%d][%d] = {\n'; % start the C array definition
fmt_row = [repmat('%.17e, ', 1, numel(variables) - 1) '%.17e']; % format for each row of the array
fmt_end = '};'; % end the C array definition

c_code = sprintf(fmt, size(coeffmap, 1), size(coeffmap, 2)); % start the C code
for i = 1:size(coeffmap, 1)
    c_code = [c_code sprintf('  {') sprintf(fmt_row, coeffmap(i, :)) sprintf('},\n')]; % add brackets for each row
end
c_code = [c_code fmt_end]; % end the C code

disp(c_code);
% Print hessian
fmt = 'static matrix_t hessian_map[%d][%d] = {\n'; % start the C array definition
fmt_row = [repmat('%.17e, ', 1, 7) '%.17e']; % format for each row of the array
fmt_end = '};'; % end the C array definition

c_code = sprintf(fmt, 8, 8); % start the C code
for i = 1:8
    c_code = [c_code sprintf('  {') sprintf(fmt_row, hf(i, :)) sprintf('},\n')]; % add brackets for each row
end
c_code = [c_code fmt_end]; % end the C code

disp(c_code);

return;

fmt=['static const double gradient_map[][]={' repmat('%d,',numel(df),numel(variables)) '}'];
c_code=sprintf(fmt,coeffmap)

