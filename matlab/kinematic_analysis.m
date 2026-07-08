clc; clear; close all;

%% 1. Input Constants
L1 = 13.5142;       % Ground (Frame) [cm]
L2 = 10.0;          % Crank [cm]
L3 = 16.8117;       % Coupler [cm]
L4 = 20.0;          % Rocker [cm]

theta2_deg = 30;    % Input Crank Angle [degrees]
omega2 = -4.338680455; % Input Angular Velocity [rad/s]
r_G3 = L3 / 2;
delta = 0;          % Offset angle (assumed 0 for uniform rod)

theta2 = deg2rad(theta2_deg);
fprintf('Processing Kinematics for Theta2 = %.2f deg...\n', theta2_deg);
fprintf('--------------------------------------------------\n');

%% 2. Position Analysis (The Diagonal Method)
% Step 1: Calculate Diagonal Length (l_BD) - Eq(1)
l_BD = sqrt(L1^2 + L2^2 - 2*L1*L2*cos(theta2));

% Step 2: Calculate Intermediate Angles (alpha, beta, psi) - Eq(2) & Eq(3)
% Beta (Angle A-O4-O2)
beta_val = acos((L1^2 + l_BD^2 - L2^2) / (2 * L1 * l_BD));
% Psi (Angle A-O4-B)
psi_val = acos((L4^2 + l_BD^2 - L3^2) / (2 * L4 * l_BD));

% Step 3: Determine Theta4 and Theta3 - Eq(4) & Eq(5)
% Theta 4 (Open configuration)
theta4 = pi - (beta_val + psi_val);

% Theta 3 (Using Sine rule rearrangement)
% Note: Standard asin has range limits; assuming valid assembly mode here.
sin_theta3 = (L4*sin(theta4) - L2*sin(theta2)) / L3;
theta3 = asin(sin_theta3);

% Quadrant Check Logic (Required for 0-360 consistency)
% If the mechanism is in the upper quadrant, theta3 calculation holds.
% If calculation yields complex numbers, linkage is impossible.
if ~isreal(theta3)
    error('Linkage cannot be assembled at this angle.');
end

% Display Position Results
fprintf('Position Analysis:\n');
fprintf('  Diagonal (l_BD): %.4f cm\n', l_BD);
fprintf('  Theta 3:         %.4f rad (%.2f deg)\n', theta3, rad2deg(theta3));
fprintf('  Theta 4:         %.4f rad (%.2f deg)\n', theta4, rad2deg(theta4));
fprintf('\n');

%% 3. Velocity Analysis
% Analytical Calculation of omega3 and omega4 - Eq(10) & Eq(11)
denominator = sin(theta3 - theta4); % Common denominator

% Equation 10
omega3 = (L2 * omega2 * sin(theta4 - theta2)) / (L3 * denominator);

% Equation 11
omega4 = (L2 * omega2 * sin(theta3 - theta2)) / (L4 * denominator);

fprintf('Velocity Analysis:\n');
fprintf('  Omega 3:         %.4f rad/s\n', omega3);
fprintf('  Omega 4:         %.4f rad/s\n', omega4);
fprintf('\n');

%% 4. Coupler COM Velocity (v_G3)
% Velocity of point A (Crank Tip)
v_Ax = -L2 * omega2 * sin(theta2);
v_Ay =  L2 * omega2 * cos(theta2);

% Velocity of Coupler COM - Eq(12 expanded)
v_G3x = v_Ax - r_G3 * omega3 * sin(theta3 + delta);
v_G3y = v_Ay + r_G3 * omega3 * cos(theta3 + delta);

% Magnitude - Eq(13)
v_G3_mag = sqrt(v_G3x^2 + v_G3y^2);

fprintf('Coupler COM Velocity:\n');
fprintf('  v_Ax:            %.4f cm/s\n', v_Ax);
fprintf('  v_Ay:            %.4f cm/s\n', v_Ay);
fprintf('  v_G3x:           %.4f cm/s\n', v_G3x);
fprintf('  v_G3y:           %.4f cm/s\n', v_G3y);
fprintf('  v_G3:            %.4f cm/s\n', v_G3_mag);
fprintf('\n');