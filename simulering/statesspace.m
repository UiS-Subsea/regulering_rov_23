
syms x y z roll pitch yaw u v w p q r du0 du1 du2 du3 du4 du5 % state and control input
syms mass Ix Iy Iz Ixy Ixz Iyz mzg % Mrb matrix
syms Xu_dot Yv_dot Zw_dot Kp_dot Mq_dot Nr_dot Xq_dot Yp_dot % Ma matrix
syms Xu Xuu Yv Yvv Zw Zww Kp Kpp Mq Mqq Nr Nrr % Damping matrices
syms gx gy gz bx by bz gravity radius water_density % Gravity and boyancy matrix

pose = [x y z roll pitch yaw];
vel = [u v w p q r];
state = sym(zeros(12,1));
state(1:6) = transpose(pose);
state(7:12) = transpose(vel);

du = [du0 du1 du2 du3 du4 du5];

gravity_center = [gx gy gz];
buoyancy_center = [bx by bz];

Mrb = sym([mass,  0.0,  0.0, 0.0, mzg, 0.0;
            0.0, mass,  0.0,-mzg, 0.0, 0.0;
            0.0,  0.0, mass, 0.0, 0.0, 0.0;
            0.0, -mzg,  0.0,  Ix, Ixy, Ixz;
            mzg,  0.0,  0.0, Ixy,  Iy, Iyz;
            0.0,  0.0,  0.0, Ixz, Iyz, Iz ]);

Ma = sym([Xu_dot,      0,      0,      0, Xq_dot,      0;
               0, Yv_dot,      0, Yp_dot,      0,      0;
               0,      0, Zw_dot,      0,      0,      0;
               0, Yp_dot,      0, Kp_dot,      0,      0;
          Xq_dot,      0,      0,      0, Mq_dot,      0;
               0,      0,      0,      0,      0, Nr_dot]);

linear_damping = diag([-Xu,-Yv,-Zw,-Kp,-Mq,-Nr]);
quadratic_damping = diag([-Xuu,-Yvv,-Zww,-Kpp,-Mqq,-Nrr]);

thruster_position = [];
thruster_direction = [];

M = sym(Mrb + Ma);
C = sym(coriolisMatrix(M,state));
D = sym(linear_damping + quadratic_damping);
G = sym(gravityMatrix(state,mass,gravity,radius,water_density,gravity_center,buoyancy_center));

%% sub funksjoner 
function [ret] = s(vec)
    % lager 3x3 antisymetrisk matrise fra 3 elements vektor
    % side 20
    ret = [   0,-vec(3), vec(2);
         vec(3),      0,-vec(1);
        -vec(2), vec(1),     0];
end

function [C] = coriolisMatrix(M,state)
    % side 53 Handbook of Marine Craft, 2011
    v1 = state(7:9);
    v2 = state(10:12);

    s1 = s(M(1:3,1:3)*v1 + M(1:3,4:6)*v2);
    s2 = s(M(4:6,1:3)*v1 + M(4:6,4:6)*v2);
    C(1:3,4:6)=-s1;
    C(4:6,1:3)=-s1;
    C(4:6,4:6)=-s2;
end

function [G] = gravityMatrix(state,mass,gravity,radius,water_density,gravity_center,buoyancy_center)
    % side 60 Handbook of Marine Craft, 2011
    phi = state(4);
    theta = state(5);
    psi = state(6);
    
    W = mass*gravity;
    F_buoyancy = ((4/3)*pi*radius^3)*water_density*gravity;

    gx = gravity_center(1);
    gy = gravity_center(2);
    gz = gravity_center(3);

    bx = buoyancy_center(1);
    by = buoyancy_center(2);
    bz = buoyancy_center(3);

    G = [(W-F_buoyancy)*sin(theta);
        -(W-F_buoyancy)*cos(theta)*sin(phi);
        -(W-F_buoyancy)*cos(theta)*cos(phi);
        -(gy*W - by*F_buoyancy)*cos(theta)*cos(phi) + (gz*W - bz*F_buoyancy)*cos(theta)*sin(phi);
        (gz*W - bz*F_buoyancy)*sin(theta) + (gx*W - bx*F_buoyancy)*cos(theta)*cos(phi);
        -(gx*W - bx*F_buoyancy)*cos(theta)*sin(phi) - (gy*W - by*F_buoyancy)*sin(theta)];
end

function [ret] = J(state)
    % transpose from body to ned
    phi = state(4);
    theta = state(5);
    psi = state(6);

    vel_NED = eul2rotm(phi,theta,psi);
    angvel_NED = [1.0,sin(phi)*tan(theta),cos(phi)*tan(theta);
                  0.0,cos(phi),-sin(phi);
                  0.0,sin(phi)/cos(theta),cos(phi)/cos(theta)];

    ret = sym(zeros(6,6));
    ret(1:3,1:3)=vel_NED;
    ret(4:6,4:6)=angvel_NED;
end
