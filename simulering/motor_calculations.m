clear all; close all;
addpath("./motor_performance");

%% PARAMETERS
% tau = [ jag, svai, hiv, rull, sett, drei]
tau_wanted = [-95, 0, 0, 0, 0, 0];
tau_wanted = [125, 0, 0, 0, 0, 0];
tau_wanted = [0, 65, 0, 0, 0, 0];
tau_wanted = [0, -65, 0, 0, 0, 0];
tau_wanted = [0, 0, 115, 0, 0, 0];
tau_wanted = [0, 0, -145, 0, 0, 0];
tau_wanted = [0, 0, 0,350, 0, 0];
tau_wanted = [0, 0, 0,-345, 0, 0];
tau_wanted = [0, 0, 0, 0, 390, 0];
tau_wanted = [0, 0, 0, 0, -390, 0];
tau_wanted = [0, 0, 0, 0, 0, 425];
tau_wanted = [0, 0, 0, 0, 0, -410];
tau_wanted = [10, 0, 80, 0, 0, 0];


COM = [-0.001,-23.016,-36.779] ./ 10;% center of mass [cm]
mass = 35840.393 / 1000; % [Kg]

% thruster positions, one row per thruster, [px,py,pz] [cm]
MPOS = [];
MPOS(1,:) = [155.15,-290.15,-52];
MPOS(2,:) = [155.15,290.15,-52];
MPOS(3,:) = [-155.15,290.15,-52];
MPOS(4,:) = [-155.15,-290.15,-52];

% vertical thrusters
MPOS(5,:) = [315,-275,-182.5];
MPOS(6,:) = [315,275,-182.5];
MPOS(7,:) = [-315,275,-182.5];
MPOS(8,:) = [-315,-275,-182.5];

% bruk thruster_data_interpolate for å finne nye verdier av disse
% verdier for 250w
umax = 40; % N
umin = -31.4; % N

total_max_power = 1000;

%% Calculate B matrix thruster-force-torque

for j=1:3
    for i=1:8
        MPOS(i,j) = MPOS(i,j) - COM(j);
    end

    COM(j) = 0;
end

return;

% konvertere motorkonfigurasjon og com fra cm til m
MPOS = MPOS ./ 100;
COM = COM ./ 100;

B = [
%   M1, M2, M3 ... M8
    cosd(33), cosd(-33), cosd(33), cosd(-33), 0, 0, 0, 0; %Fx
    sind(33), sind(-33), sind(33), sind(-33), 0, 0, 0, 0; %Fy
    0,0,0,0,-1,-1,-1,-1 %Fz
];

for i=1:8
   radius = MPOS(i,:) - COM;
   F_uvec = B(1:3,i);
   B(4:6,i) = cross(radius ,F_uvec)';
end
% u = [3.80, 37.93, 10.39, 31.34, 0.17, -4.91, -0.17, 4.91]';
% tau = B*u

% for printing B matrix in c code
print_B = false;
if print_B
    % Corrected format string
    fmt = 'static matrix_t B[%d][%d] = {\n'; % start the C array definition
    fmt_row = [repmat('%.17e, ', 1, 7) '%.17e']; % format for each row of the array
    fmt_end = '};'; % end the C array definition
    
    c_code = sprintf(fmt, 6, 8); % start the C code
    for i = 1:6
        c_code = [c_code sprintf('  {') sprintf(fmt_row, B(i, :)) sprintf('},\n')]; % add brackets for each row
    end
    c_code = [c_code fmt_end]; % end the C code
    
    disp(c_code);
    return;
end

%% Load tabledata and generate conversions
tabledata = readtable("T200_data_16v.csv");
td_pwm = tabledata{:,1};
td_percent = (td_pwm - 1500)/4;
td_power = tabledata{:,5};
td_force = tabledata{:,6};
td_force = td_force .* 9.80665; % Convertere fra Kgf til newton
td_force_abs = abs(td_force);

% approximate newton/power and generate symbolic function
degree = 30;
ffit_forcepower = polyfit(td_force, td_power,degree);
ffit_forcepercent = polyfit(td_force,td_percent,degree);
% figure;
% hold on;
% fplot(poly2sym(a));
% plot(td_force,td_percent);
% hold off;
% plot approximation to check if ok
%hold on;
%fplot(P)
%plot(td_force,td_power);
%xlim([umin umax]);
%hold off;

%% Optimization
% Weights
Kf = 1e50;
Kt = 1e50;
Kpwr = 1e10;

Fx = tau_wanted(1);
Fy = tau_wanted(2);
Fz = tau_wanted(3);
Tx = tau_wanted(4);
Ty = tau_wanted(5);
Tz = tau_wanted(6);

% Define the objective function
objFun = @(U) Kf * (Fx - sum(B(1, :) .* U))^2 + Kf * (Fy - sum(B(2, :) .* U))^2 + Kf * (Fz - sum(B(3, :) .* U))^2 + ...
    Kt * (Tx - sum(B(4, :) .* U))^2 + Kt * (Ty - sum(B(5, :) .* U))^2 + Kt * (Tz - sum(B(6, :) .* U))^2 + ...
    Kpwr * sum(U.^2);

% Define constraints
nonlcon = @(U) constraintFunction(U, umin, umax, ffit_forcepower, degree,total_max_power);

% Initial guess for U
initialGuess = zeros(1, 8);

lb = ones(1,8) * umin;
ub = ones(1,8) * umax;

% Use fmincon to find the optimal values of U
options = optimoptions('fmincon','Algorithm','sqp','MaxIterations',1e2);
options = optimoptions(options,'MaxIterations',1e2); % Recommended
options = optimoptions(options, 'Display', 'iter');
optimalU = fmincon(objFun, initialGuess, [], [], [], [], lb, ub, nonlcon, options);
optimalU_percent = polyval(ffit_forcepercent,optimalU);

disp('Optimal values for U in Newtons:');
disp(optimalU);

disp('Optimal values for U in Percent:');
disp(optimalU_percent);


% Calculate the corresponding forces and torques using the optimal U
rFx = B(1, :) * optimalU';
rFy = B(2, :) * optimalU';
rFz = B(3, :) * optimalU';
rTx = B(4, :) * optimalU';
rTy = B(5, :) * optimalU';
rTz = B(6, :) * optimalU';

tau_result = [rFx rFy rFz rTx rTy rTz];

disp("Gives tau:");
disp(tau_result);

%% Calculate velocity and rotation calculations
m = 50;
s = 1;
I = m*s^2/6;
Ix = I; Iy = I; Iz = I;

aaX = rTx/Ix;
aaY = rTy/Iy;
aaZ = rTz/Iz;

Ps([66.0983   65.5247   65.5258   66.0747  -51.1390  -51.8631  -64.7005  -66.5883], ffit_forcepower, degree)

function power = P(x, ffit, degree)
    % Evaluates the polynomial at x
    power = 0;
    for i = 1:degree+1
        power = power + ffit(i)*(x)^(degree-i+1);
    end
    if power < 0
        power = 0;
    end
end

function power_sum = Ps(U, ffit, degree)
    % Calculates the sum of powers for the given U
    power_sum = 0;
    for i = 1:length(U)
        power_sum = power_sum + P(U(i), ffit, degree);
    end
end

function [c, ceq] = constraintFunction(U, umin, umax, ffit, degree,total_max_power)
    % Define the inequality and equality constraints
    c = [U - umax, umin - U, Ps(U,ffit,degree) - total_max_power];
    ceq = [];
end