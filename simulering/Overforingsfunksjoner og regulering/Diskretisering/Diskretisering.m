%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Diskretisering %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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



%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Diskretisering %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

z = tf('z');
Ts = 0.1; %sampletime


H_ZOH= c2d(H_hiv_fart,Ts,'zoh');

H_FOH=c2d(H_hiv_fart,Ts,'foh');

H_Impulse=c2d(H_hiv_fart,Ts,'impulse');

H_Tustin=c2d(H_hiv_fart,Ts,'tustin');

H_Matched=c2d(H_hiv_fart,Ts,'matched');

%bode(H_phi,H_ZOH, H_FOH,H_Impulse,H_Tustin,H_Matched)
%impulse(H_hiv_fart,H_ZOH, H_FOH,H_Impulse,H_Tustin,H_Matched, 2)

%impulse(H_phi, H_Matched, H_Tustin, 1)

%pzmap(H_hiv_fart,H_ZOH, H_FOH,H_Impulse,H_Tustin,H_Matched)
%legend("H(s)", "H ZOH", "H FOH","H Impulse","H Tustin", "H Mached")

%rlocus(H_FOH)



%%%%%% sampletime effekt %%%%%%

Ts001=0.01;
Ts01 = 0.1;
Ts02 = 0.2;
Ts1 = 1;
Ts4 = 4;


H_Tustin001=c2d(H_hiv_fart,Ts001,'tustin');
H_Tustin01=c2d(H_hiv_fart,Ts01,'tustin');
H_Tustin02=c2d(H_hiv_fart,Ts02,'tustin');
H_Tustin1=c2d(H_hiv_fart,Ts1,'tustin');
H_Tustin4=c2d(H_hiv_fart,Ts4,'tustin');


%pzmap(H_hiv_fart, H_Tustin001, H_Tustin01, H_Tustin02, H_Tustin1, H_Tustin4)
%impulse(H_hiv_fart, H_Tustin001, H_Tustin01, H_Tustin02, H_Tustin1, H_Tustin4, 20)
%legend("Kont", "Ts 0.01", "Ts 0.1", "Ts 0.2","Ts 1", "Ts 4")


%%%%%% overføring og PI samla (Heq) %%%%%%

P = 1524;
I = 7494;

N_PI_hiv=  [P I];
D_PI_hiv = [1 0];
H_PI_hiv= tf(N_PI_hiv,D_PI_hiv)
H_PI_hiv_z = c2d(H_PI_hiv,Ts,"tustin")

H_eq= feedback(H_PI_hiv*H_hiv_fart,1);
H_eq_z=c2d(H_eq,Ts,"tustin");

step(H_eq_z, H_eq)

H_eq_z_001=c2d(H_eq,Ts001,'tustin');
H_eq_z_01=c2d(H_eq,Ts01,'tustin');
H_eq_z_02=c2d(H_eq,Ts02,'tustin');
H_eq_z_1=c2d(H_eq,Ts1,'tustin');
H_eq_z_4=c2d(H_eq,Ts4,'tustin');

