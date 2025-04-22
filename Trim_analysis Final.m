%parameters
phi = 0; % Roll angle (rad)
alpha = deg2rad(2); % Angle of attack (rad)
g = 9.80665; % Gravity (m/s^2)
m = 2884.8475; % MTOM (kg)
Vinf = 139; % Velocity (m/s)
rho = 0.3025; % Air density (kg/m^3)
S = 55.47; % Wing area (m^2)
b = 1.524; % Wingspan (m)
T = 1200; % Thrust per engine (N)
phi_T = deg2rad(5); % Engine incidence angle (rad)
k_OEI = 1.15; % Inactive engine drag factor (-)
y_T = 2.5; % Engine lateral position (m)

% Aerodynamic Coefficients
Cy_b = -0.346;
Cy_da = 0;
Cy_dr = 0.2;
Cl_b = -0.0944;
Cl_da = 0.18;
Cl_dr = 0.015;
Cn_b = 0.11;
Cn_da = -0.025;
Cn_dr = -0.036;

% Pitching Moment Coefficients (for delta_e calculation)
cm_0 = -0.025;        % Pitching moment at zero angle of attack
cm_alpha = -0.7;     % Pitching moment per radian of alpha
cm_delta_e = -1.12;   % Pitching moment per radian of elevator deflection
% -----------------delta_e for Cruise Condition -----------------
delta_e = - (cm_0 + cm_alpha * alpha) / cm_delta_e;
delta_e_deg = rad2deg(delta_e);

disp('Elevator Deflection (delta_e) in Cruise Condition:');
disp(['delta_e (deg): ', num2str(delta_e_deg)]);

% ------------------------- Engine failure -------------------------
% Dynamic Pressure
Q = rho * Vinf^2 / 2; % Dynamic pressure (Pa)
dT = 1.15 * T; % Thrust differential, Engine 1 inoperative (N)

% Aerodynamic Coefficient Matrix
Ac = [Cy_b Cy_da Cy_dr; Cl_b Cl_da Cl_dr; Cn_b Cn_da Cn_dr];

% Output vector for engine failure
Y = [0; -y_T * sin(phi_T + alpha) * dT / (Q * S * b); 
       y_T * cos(phi_T + alpha) * dT / (Q * S * b)];

% Cramer's rule for engine failure
beta = det([Y(:), Ac(:,2:3)]) / det(Ac);
delta_a = det([Ac(:,1), Y(:), Ac(:,3)]) / det(Ac);
delta_r = det([Ac(:,1:2), Y(:)]) / det(Ac);
disp('Cramer rule [beta; delta_a; delta_r] (deg) = ');
disp(rad2deg([beta; delta_a; delta_r]));

% Inverse matrix method for engine failure
sol = inv(Ac) * Y;
disp('Trim needed to counteract the engine failure [beta; delta_a; delta_r] (deg) = ');
disp(rad2deg(sol));




