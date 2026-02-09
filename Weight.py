%% =====================================================
% CONSTANTS
%% =====================================================
g   = 9.81;
rho = 1.225;

%% =====================================================
% PAYLOAD
%% =====================================================
Wp = 1.75;   % kg

%% =====================================================
% EMPTY WEIGHT REGRESSION (Raymer)
%% =====================================================
a = 0.524;
l = -0.029;
We_frac_min = 0.60;   % VTOL fixed-wing realistic lower bound

%% =====================================================
% BATTERY
%% =====================================================
specific_energy = 240;   % Wh/kg
usable_frac = 0.75;
reserve_frac = 0.30;

%% =====================================================
% AERODYNAMICS
%% =====================================================
S  = 0.4;
Vc1 = 18;   % cruise
Vc2 = 12;   % loiter cruise
Vt  = 12;   % turn speed
R   = 65;   % turn radius

CD0 = 0.035;
AR  = 10;
e   = 0.8;
k   = 1/(pi*AR*e);
eta_prop = 0.65;

%% =====================================================
% MISSION TIMES
%% =====================================================
t_cruise1 = 230;
t_cruise2 = 3000;
t_turn    = 340;
t_vtol    = 200;

%% =====================================================
% VTOL ROTOR MODEL
%% =====================================================
DL = 100;          % N/m^2
k_induced = 1.15;
profile_coeff = 0.012;

%% =====================================================
% ITERATION SETTINGS
%% =====================================================
W0 = 12.0;         % initial guess (kg)
tol = 1e-3;
max_iter = 200;

%% =====================================================
% MTOW FIXED-POINT ITERATION
%% =====================================================
for i = 1:max_iter

    W = W0 * g;

    %% ---- EMPTY WEIGHT FRACTION ----
    We_frac = max(a * W0^l, We_frac_min);

    %% ---- VTOL POWER ----
    A_total = W / DL;
    vi = sqrt(W / (2 * rho * A_total));
    P_hover = k_induced * W * vi + ...
              profile_coeff * rho * A_total * vi^3;
    E_vtol = P_hover * t_vtol / 3600;

    %% ---- CRUISE POWER (18 m/s) ----
    CL1 = (2 * W) / (rho * Vc1^2 * S);
    CD1 = CD0 + k * CL1^2;
    D1  = 0.5 * rho * Vc1^2 * S * CD1;
    E1  = (D1 * Vc1 / eta_prop) * t_cruise1 / 3600;

    %% ---- LOITER CRUISE POWER (12 m/s) ----
    CL2 = (2 * W) / (rho * Vc2^2 * S);
    CD2 = CD0 + k * CL2^2;
    D2  = 0.5 * rho * Vc2^2 * S * CD2;
    E2  = (D2 * Vc2 / eta_prop) * t_cruise2 / 3600;

    %% ---- TURN POWER (COORDINATED TURN) ----
    n = sqrt(1 + (Vt^2 / (g * R))^2);
    CLt = n * (2 * W) / (rho * Vt^2 * S);
    CDt = CD0 + k * CLt^2;
    Dt  = 0.5 * rho * Vt^2 * S * CDt;
    Et  = (Dt * Vt / eta_prop) * t_turn / 3600;

    %% ---- TOTAL ENERGY ----
    E_total = (E_vtol + E1 + E2 + Et) * (1 + reserve_frac);

    %% ---- BATTERY WEIGHT ----
    Wb = E_total / (specific_energy * usable_frac);
    Wb_frac = Wb / W0;

    %% ---- RAYMER MTOW UPDATE (NO RELAXATION) ----
    W0_new = Wp / (1 - We_frac - Wb_frac);

    %% ---- CONVERGENCE CHECK ----
    if abs(W0_new - W0)/W0 < tol
        W0 = W0_new;
        break
    end

    W0 = W0_new;
end

%% =====================================================
% RESULTS
%% =====================================================
fprintf('\n===== RAYMER VTOL UAV (FIXED-POINT) =====\n');
fprintf('MTOW               : %.3f kg\n', W0);
fprintf('Empty weight frac  : %.3f\n', We_frac);
fprintf('Empty weight       : %.3f kg\n', We_frac * W0);
fprintf('Battery weight     : %.3f kg\n', Wb);
fprintf('Battery frac       : %.3f\n', Wb_frac);
fprintf('Payload frac       : %.3f\n', Wp / W0);
fprintf('Total Energy       : %.1f Wh\n', E_total);
fprintf('Iterations         : %d\n', i);

