% EOM Linearization for SS Model

% Aircraft Parameters
m = 2890;    % kg
g = 9.8;    % m/s^2
Ixx = 10820; % kg*m^2
Iyy = 4507; % kg*m^2
Izz = 15153; % kg*m^2
Ixz = 0; % kg*m^2
S = 16.7; % m^2
Q = 4438; % Pa
b = 10.3; % m
c = 1.66; % m
yT1 = b/4; % m
dT = 0; % m
phiT = 0; % rad

% Aerodyamic Coefficients
CD_0 = 0.02;
CD_alpha = 0.25;
CD_ih = 0;
CD_de = 0;
CY_beta = -0.346;
CY_da = 0;
CY_dr = 0.2;
CL_0 = 0.2;
CL_alpha = 5.15;
CL_ih = 0;
CL_de = 0.5;
Cl_beta = -0.0944;
Cl_da = 0.181;
Cl_dr = 0.015;
Cm_0 = 0.025;
Cm_alpha = -0.7;
Cm_ih = 0;
Cm_de = -1.12;
Cn_beta = 0.1106;
Cn_da = -0.0254;
Cn_dr = -0.0365;


% System Parameters
u_cruise = 139; % m/s
T1 = 1078; % N
T2 = T1; % N
alpha = deg2rad(2); % rad
beta = 0; % rad
z_cruise = -9146; % m

syms X [12 1] real
syms U [4 1] real

% State Variable and Inputs Legend
xe = X1;
ye = X2;
ze = X3;
phi = X4;
theta = X5;
psi = X6;
u = X7;
v = X8;
w = X9;
p = X10;
q = X11;
r = X12; 

ih = U1;
de = U2;
da = U3;
dr = U4;

% Define external forces and moments
Fxb = -m*g*sin(theta+alpha) -Q*S*(CD_0+CD_alpha*alpha+CD_ih*ih+CD_de*de) + (T1+T2)*cos(phiT+alpha);
Fyb = m*g*cos(theta+alpha)*sin(phi) + Q*S*(CY_beta*beta+CY_da*da+CY_dr*dr) + 0;
Fzb = m*g*cos(phi)*cos(theta+alpha) -Q*S*(CL_0+CL_alpha*alpha+CL_ih*ih+CL_de*de) - (T1+T2)*sin(phiT+alpha);
Mxb = 0 + Q*S*b*(Cl_beta*beta+Cl_da*da+Cl_dr*dr) + yT1*sin(phiT+alpha)*(T2-T1);
Myb = 0 + Q*S*c*(Cm_0+Cm_alpha*alpha+Cm_ih*ih+Cm_de*de) + dT*(T1+T2);
Mzb = 0 + Q*S*b*(Cn_beta*beta+Cn_da*da+Cn_dr*dr) + yT1*cos(phiT+alpha)*(T2-T1);

% Nonlinear EOMs (Simplified for Ixz=Ixz=0)
xe_dot = u*cos(psi)*cos(theta)+v*(cos(psi)*sin(phi)*sin(theta)-cos(phi)*sin(psi))+w*(sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta));
ye_dot = u*cos(theta)*sin(phi)+v*(cos(phi)*cos(psi)+sin(phi)*sin(psi)*sin(theta))+w*(cos(phi)*sin(psi)*sin(theta)-cos(psi)*sin(phi));
ze_dot = -u*sin(theta)+v*cos(theta)*sin(phi)+w*cos(phi)*cos(theta);
phi_dot = p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);
theta_dot = q*cos(phi)-r*sin(phi);
psi_dot = q*(sin(phi)/cos(theta))+r*(cos(phi)/cos(theta));
u_dot = r*v-q*w + Fxb/m;
v_dot = p*w-r*u + Fyb/m;
w_dot = q*u-p*v + Fzb/m;
p_dot = -((Izz^2+Iyy*Izz)*q*r+Izz*Mxb)/(Ixx*Izz);
q_dot = ((Izz-Ixx)*p*r+Myb)/(Iyy);
r_dot = ((Ixx^2-Ixx*Iyy)*p*q+Ixx*Mzb)/(Ixx*Izz);


% Combine into 1 matrix
f_EOM = [xe_dot; ye_dot; ze_dot; phi_dot; theta_dot; psi_dot; u_dot; v_dot; w_dot; p_dot; q_dot; r_dot;];

% Equilibrium point
Xeq = [0; 0; 0; 0; 0; 0; u_cruise; 0; 0; 0; 0; 0;];
Ueq = [0; 0; 0; 0;];

% Compute Jacobians
A = jacobian(f_EOM, X);  % Partial derivative w.r.t. state variables X
B = jacobian(f_EOM, U);  % Partial derivative w.r.t. inputs U

% Evaluate at the equilibrium point
A_at_eq = subs(A, [X; U], [Xeq; Ueq]);
B_at_eq = subs(B, [X; U], [Xeq; Ueq]);

% Simplify/Evaluate
A_at_eq = double(A_at_eq);
B_at_eq = double(B_at_eq);

% Transfer Function
C_tf = eye(12);
D_tf = zeros(12, 4);
system = ss(A_at_eq, B_at_eq, C_tf, D_tf);
TF = tf(system);

% LQR Controller    (Couldnt get to work, didnt use)
% Q_lqr = eye(12);   % State cost matrix
% R_lqr = eye(4);    % Control cost matrix
% Calculate LQR gains
% [K_lqr,~,~] = lqr(A_at_eq, B_at_eq, Q_lqr, R_lqr);
% disp(K_lqr);

% PID Controller

aero = [CD_ih 0 CL_ih 0 Cm_ih 0 CD_ih 0 CL_ih 0 Cm_ih 0;    % U1 ih
        CD_de 0 CL_de 0 Cm_de 0 CD_de 0 CL_de 0 Cm_de 0;    % U2 de
        0 CY_da 0 Cl_da 0 Cn_da 0 CY_da 0 Cl_da 0 Cn_da;    % U3 da
        0 CY_dr 0 Cl_dr 0 Cn_dr 0 CY_dr 0 Cl_dr 0 Cn_dr];   % U4 dr
      % xe   ye  ze  phi  theta  psi  u  v  w  p  q  r

aero0 = -1*[0 0 0 0 0 0 CD_ih 0 CL_ih 0 Cm_ih 0;    % U1 ih
            0 0 0 0 0 0 CD_de 0 CL_de 0 Cm_de 0;    % U2 de
            0 0 0 0 0 0 0 CY_da 0 Cl_da 0 Cn_da;    % U3 da
            0 0 0 0 0 0 0 CY_dr 0 Cl_dr 0 Cn_dr];   % U4 dr
         % xe ye ze phi theta psi u v w p q r

aero1 = [1 1 1 1 1 1 1 1 1 1 1 1;   % U1 ih
        1 1 1 1 1 1 1 1 1 1 1 1;    % U2 de
        1 1 1 1 1 1 1 1 1 1 1 1;    % U3 da
        1 1 1 1 1 1 1 1 1 1 1 1;];  % U4 dr
       % xe ye ze phi theta psi u v w p q r

aero2 = [1 0 1 0 2 0 1 0 1 0 2 0;   % U1 ih
        1 0 1 0 2 0 1 0 1 0 2 0;    % U2 de
        0 1 0 3 0 0 1 0 0 3 0 0;    % U3 da
        2 2 0 0 0 3 1 1 0 0 0 3;];  % U4 dr
       % xe ye ze phi theta psi u v w p q r

aero3 = -1*[0 0 0 0 0 0 1 0 1 0 2 0;   % U1 ih
        0 0 0 0 0 0 1 0 1 0 2 0;    % U2 de
        0 0 0 0 0 0 1 0 0 3 0 0;    % U3 da
        0 0 0 0 0 0 1 1 0 0 0 3;];  % U4 dr
       % xe ye ze phi theta psi u v w p q r

Kp = 1*aero0;
Ki = 0.1*aero0;
Kd = -0.001*aero0;


% Simulation
X_ic = [0; 0; z_cruise; 0; 0; 0; u_cruise; 0; 0; 0; 0; 0];
ourSim = sim("AircraftControlSystem");



%{
% Plotting
% Get actual and reference arrays of x, y, z from simulink model
ourSim = sim("AircraftControlSystem");
xpos = ourSim.xposition;
ypos = ourSim.yposition;
zpos = ourSim.zposition;
xr = ourSim.xref;
yr = ourSim.yref;
zr = ourSim.zref;

% Plot Results
figure;
plot3(xpos,ypos,zpos,'b-');
hold on;
plot3(xr,yr,zr,'r--');
grid on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('3D Trajectory');
legend('Actual Trajectory','Desired Trajectory');
hold off;

%}

