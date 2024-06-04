%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Lead Lag regulereing %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear

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
% max x 125N, 0.89 m/s 

%%% Hiv %%%

v_max_hiv = 0.65; % maks fart [m/s]
v_5_hiv = 0.05*v_max_hiv; % for linarisering ved lav hastighet, 5% av maks fart
v_70_hiv = 0.7 *v_max_hiv; % for linarisering ved høg hastighet, 70% av maks fart
Z_2= v_5_hiv; % linariseringspunkt

N_hiv_fart = 1; %teller hiv, fart
D_hiv_fart = [M(3),rho_vann*Cd*A_z*Z_2]; % nevner hiv, fart
H_hiv_fart = tf(N_hiv_fart, D_hiv_fart);


%%%%%%%%%%%%%%%%%%
%%%% LEAD LAG %%%%
%%%%%%%%%%%%%%%%%%

P = 1524;
I = 7494;

N_PI_hiv= [P I];
D_PI_hiv = [1 0];
H_PI_hiv= tf(N_PI_hiv,D_PI_hiv);


N_lead = [29000, 29000*18];
D_lead = [1, 180];

H_lead = tf(N_lead, D_lead);

N_lag = [29000 29000*0.12];
D_lag= [1 0.012];
H_lag= tf(N_lag, D_lag);

bode(H_lag, H_lead, H_PI_hiv)



