%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Frå diskre til differanse %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
close all

%%%% Generelle parameter, same som for Dynamikk.m %%%%

tau = [125;0;0;37.89;0;0]; % kraftbidrag frå thusterene
% max x 125N, 0.89 m/s 
% max y 65N, 0.66 m/s
% max z -145N, 0.65 m/s
% 37.89N,(F_b*r_mb), i rull gir ca 90deg. 

m = 35 ;    % masse 
l = 0.66;   % lengde
b = 0.7;    % bredde
h = 0.3;    % høgd

rho_vann = 1000;     % kg/m^3 Vannmotstand ferskvann
rho_saltvann = 1025; % kg/m^3 Vannmotstand saltvann
g = 9.81;            % m/s^2 tyngdekraftens akelerasjon
G = m*g;             % gravitasjonskraft
Cd = 1.5;            % dragkoefisent



%%%% Treghetmoment, M %%%%

J = (1/12)*m*[b^2+h^2,h^2+l^2,l^2+b^2]; % treghetsmoment

M_rb = [m,m,m,J(1),J(2),J(3)]; % moment og dreiemoment
M_A = [35, 20, 50, 1,1,1 ];    % added mass  

M = M_rb + M_A;



%%%% Vannmotstand overflate rotasjon, D %%%%

A_x = b*h;  % overflateareal jag
A_y = l*h;  % overflateareal svai
A_z = b*l;  % overflateareal hiv
A_phi = A_y + A_z;      % overflateareal rull
A_thetha = A_x + A_z;   % overflateareal stamp
A_psi = A_x + A_y;      % overflateareal gir

A = [A_x, A_y, A_z, A_phi, A_thetha, A_psi];



%%%% Oppdrift og tyngekraft, G %%%%

COM = [ 0, -7.5, 30];   % center of mass
Zm = COM(3)-COM(3);     % flytter z-koordinat for å bruke COM som referanse
COB = [ 0, -7.5, 67.9]; % center of boyency
Zb = COB(3)-COM(3);     % flytter Zb relativ til COM
r_mb = norm(COB - COM) ; % avstand COB og COM

F_b = 1; % oppdrift, 1N



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Overføringsfunksjoner %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

s = tf('s');
z = tf('z');
Ts01 = 0.01;

P = 524;
I = 3494;

N_PI_hiv= [P I];
D_PI_hiv = [1 0];
H_PI_hiv= tf(N_PI_hiv,D_PI_hiv);
H_PI_hiv_z = c2d(H_PI_hiv,Ts01,"tustin");

%%% Hiv %%%

v_max_hiv = 0.65; % maks fart [m/s]
v_5_hiv = 0.05*v_max_hiv; % for linarisering ved lav hastighet, 5% av maks fart
v_70_hiv = 0.7 *v_max_hiv; % for linarisering ved høg hastighet, 70% av maks fart
Z_2= v_5_hiv; % linariseringspunkt

N_hiv_fart = 1; %teller hiv, fart
D_hiv_fart = [M(3),rho_vann*Cd*A_z*Z_2]; % nevner hiv, fart
G_hiv = tf(N_hiv_fart, D_hiv_fart);
G_hiv_z = c2d(G_hiv, Ts01,"tustin");

H_tot = feedback(G_hiv*H_PI_hiv,1);
H_tot_z = c2d(H_tot,Ts01,'tustin');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% differanselikninger %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t_slutt = 6;
N = t_slutt/Ts01; 
step_k = 5;

u(1:step_k) = 0;
r=ones(1,N);
r(1:step_k) =0;
y(1) =0;

[num_H_PI,den_H_PI]=tfdata(H_PI_hiv_z,'v');
[num_G,den_G]=tfdata(G_hiv_z,'v');

e(1) = 0;
u(1) = 0;

for n = 2:N
    y(n) = y(n-1);
    e(n) = r(n)-y(n);
    u(n) = num_H_PI(1)*e(n) + num_H_PI(2)*e(n-1) - den_H_PI(2)*u(n-1);
    y(n) = num_G(1)*u(n) + num_G(2)*u(n-1) - den_G(2)*y(n-1);
end

t = [1:N]*Ts01;

subplot(3,1,1)
plot(t,u)
legend('u')

subplot(3,1,2)
plot(t,e)
legend('e')

subplot(3,1,3)
plot(t,r,'r')
hold on
plot(t,y,'b')
 
[y_H_tot,tt1]=step(H_tot,t_slutt);
[y_H_tot_z,tt2]=step(H_tot_z,t_slutt);
tt1 = [0; tt1(1:end-1)+Ts01*step_k];
y_H_tot = [0; y_H_tot(1:end-1)];
tt2 = [0; tt2(1:end-1)+Ts01*step_k];
y_H_tot_z = [0; y_H_tot_z(1:end-1)];

plot(tt1,y_H_tot)
plot(tt2,y_H_tot_z)

legend('r','y','step(H_tot)','step(H_tot_z)')
figure
plot(t,r,'r')

hold on
plot(t,y,'b')
plot(tt1,y_H_tot)
plot(tt2,y_H_tot_z)

legend('r','y','step(H\_tot)','step(H\_tot\_z)')

figure
step(H_tot, H_tot_z)
xlim([0 t(end)])

