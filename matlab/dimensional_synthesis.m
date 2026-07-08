clc; clear;

% 1. Known Variables
r = 10;                     % length of crank
l = 20;                     % length of rocker
theta = deg2rad(17);        % rocker angle at the "halfway stage" in radians

% 2. Define unknowns as symbolic variables
syms b lprime beta

% 3. Set up the geometric constraint equations
eq1 = b^2 == lprime^2 - (l - r)^2;
eq2 = l * cos(theta) + lprime * cos(beta) == b;
eq3 = l * sin(theta) + r == lprime * sin(beta);

% 4. Solve the system numerically
disp('--- Numerical Solutions (b, lprime, beta) ---');
solutions = vpasolve([eq1, eq2, eq3], [b, lprime, beta]);

% 5. Display the results
disp(['b = ', char(solutions.b)]);
disp(['lprime = ', char(solutions.lprime)]);
disp(['beta = ', char(solutions.beta)]);