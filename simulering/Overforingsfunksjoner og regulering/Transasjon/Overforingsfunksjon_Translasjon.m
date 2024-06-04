%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Overføringsfunksjoner Translasjon %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
% max y 65N, 0.66 m/s
% max z -145N, 0.65 m/s

%%% Hiv %%%

v_max_hiv = 0.65; % maks fart [m/s]
v_5_hiv = 0.05*v_max_hiv; % for linarisering ved lav hastighet, 5% av maks fart
v_70_hiv = 0.7 *v_max_hiv; % for linarisering ved høg hastighet, 70% av maks fart
Z_2= v_5_hiv; % linariseringspunkt

N_hiv_fart = 1; %teller hiv, fart
D_hiv_fart = [M(3),rho_vann*Cd*A_z*Z_2]; % nevner hiv, fart
H_hiv_fart = tf(N_hiv_fart, D_hiv_fart)
N_hiv_posisjon = 1; %teller hiv, posisjon
D_hiv_posisjon = [M(3),rho_vann*Cd*A_z*Z_2,0]; % nevner hiv, posisjon
H_hiv_posisjon = tf(N_hiv_posisjon, D_hiv_posisjon)


%%% Jag %%%

v_max_jag = 0.89; % maks fart [m/s]
v_5_jag = 0.05*v_max_jag; % for linarisering ved lav hastighet, 5% av maks fart
v_70_jag = 0.7 *v_max_jag; % for linarisering ved høg hastighet, 70% av maks fart
X_2= v_70_jag; % linariseringspunkt

N_jag = 1; % teller jag
D_jag = [M(1),rho_vann*Cd*A_x*X_2]; % nevner jag
H_jag = tf(N_jag, D_jag);




%%% Svai %%%

v_max_svai = 0.66; % maks fart [m/s]
v_5_svai = 0.05*v_max_svai; % for linarisering ved lav hastighet, 5% av maks fart
v_70_svai = 0.7 *v_max_svai; % for linarisering ved høg hastighet, 70% av maks fart
Y_2= v_70_svai; % linariseringspunkt

N_svai = 1; % teller jag
D_svai = [M(2),rho_vann*Cd*A_y*Y_2]; % nevner jag
H_svai = tf(N_svai, D_svai) 

