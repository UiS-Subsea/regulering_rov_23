%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Overføringsfunksjoner Rotasjon %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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



%%% Rull, phi %%%

phi_2 = 0.25;   % fart, rad/s
phi_1 = 10;     % linariseringspunkt, deg
thetha = 0;     % vinkel i stamp
 
N_phi = 1;      % teller phi
D_phi = [M(4),rho_vann*Cd*A_phi*phi_2,(Zm*G-Zb*F_b)*cosd(thetha)*cosd(phi_1)]; % nevner phi
H_phi = tf(N_phi, D_phi) % overføringsfunksjon for rull, phi 



%%% Stamp, thetha %%%

thetha_2 = 0.26; % linariseringspunkt fart
thetha_1 = 10;  % linariseringspunkt posisjon

N_thetha = 1; % teller thetha
D_thetha = [M(5),rho_vann*Cd*A_thetha*thetha_2,(Zm*G-Zb*F_b)*cosd(thetha_1)]; % nevner thetha
H_thetha = tf(N_thetha, D_thetha) % overføringsfunksjon for rull, thetha 



%%% Gir, psi %%%

psi_2= 0.26; % linariseringspunkt fart 

N_psi = 1; % teller psi
D_psi = [M(6),rho_vann*Cd*A_psi*psi_2]; % nevner gir
H_psi = tf(N_psi, D_psi) % overføringsfunksjon for rull, psi


