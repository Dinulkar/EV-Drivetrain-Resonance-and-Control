%% FlexibleShaft_parameters.m
% Flexible-shaft design driven by the rigid-model maximum torque.
%
% IMPORTANT:
% No separate motor inertia Jm is assumed or invented.
%
% The original model has no independent motor rotor inertia. Therefore the
% flexible extension preserves that level of modeling:
%
%   T_s = T_M = k*i_M
%   T_s = K_s*(theta_M-theta_R)
%
% which gives:
%   DeltaTheta = k*i_M/K_s
%   omega_M-omega_R = (k/K_s)*di_M/dt
%
% Substitution into the motor electrical equation yields:
%   (L+k^2/K_s)*di_M/dt = V_s - R*i_M - k*omega_R

run('RigidShaft_parameters.m');

if ~isfile('RigidShaft_MaxTorque.mat')
    error(['RigidShaft_MaxTorque.mat not found. ' ...
           'Run RigidShaft_MaxTorque.m first.']);
end

load('RigidShaft_MaxTorque.mat','Tmax_rigid','Tmax_case','T_rigid','f_rigid');

%% New shaft: explicit DESIGN CHOICES
% These are properties chosen for the added shaft, not original parameters.
shaft_material = 'Steel';
G = 79e9;                 % Shear modulus [Pa]
shaft_length = 0.30;      % Shaft length [m]
tau_allow = 50e6;         % Allowable shear stress [Pa]
SafetyFactor = 2.0;       % [-]

%% Shaft diameter from rigid maximum torque
T_design = SafetyFactor*Tmax_rigid;

% Solid circular shaft:
% tau_max = 16*T/(pi*d^3)
d_required = (16*T_design/(pi*tau_allow))^(1/3);

preferred_diameters_mm = [6 8 10 12 15 16 20 25 30 35 40];
idxD = find(preferred_diameters_mm >= d_required*1e3,1,'first');
if isempty(idxD)
    error('Required shaft diameter exceeds the preferred diameter list.');
end

d_selected_mm = preferred_diameters_mm(idxD);
d_selected = d_selected_mm*1e-3;

%% Torsional stiffness
Jpolar = pi*d_selected^4/32;
Ks = G*Jpolar/shaft_length;  % [Nm/rad]

%% Reduced compliant-shaft model without Jm
L_effective = L + k^2/Ks;

% States x = [i_M; omega_R]
% Inputs u = [V_s; M_L]
%
% di/dt = -(R/Leff)i - (k/Leff)omega_R + (1/Leff)V_s
% domega_R/dt = (k/Ip)i + (1/Ip)M_L

A_flexible = [-R/L_effective, -k/L_effective;
               k/Ip,            0];

B_flexible = [1/L_effective, 0;
              0,               1/Ip];

lambda_flexible = eig(A_flexible);

idx = find(imag(lambda_flexible) > 0,1);
omega_d_flexible = imag(lambda_flexible(idx));
f_flexible = omega_d_flexible/(2*pi);
T_flexible = 1/f_flexible;

%% Output relationships
% T_s          = k*i_M
% DeltaTheta   = (k/Ks)*i_M
% di_M/dt      = (V_s-R*i_M-k*omega_R)/L_effective
% omega_M      = omega_R + (k/Ks)*di_M/dt

%% Simulation settings
FlexibleStopTime = 5;
FlexibleMaxStep  = 1e-3;
FlexibleRelTol   = 1e-9;
FlexibleAbsTol   = 1e-11;

% Default = voltage-step task with M_L = 0
ML_flexible_model_amp = 0;

fprintf('\n============================================================\n');
fprintf('FLEXIBLE-SHAFT DESIGN\n');
fprintf('============================================================\n');
fprintf('Rigid-model torque basis = %.6f Nm\n',Tmax_rigid);
fprintf('Rigid case producing Tmax = %s\n',char(Tmax_case));
fprintf('Shaft material            = %s\n',shaft_material);
fprintf('G                         = %.3f GPa\n',G/1e9);
fprintf('Shaft length              = %.3f m\n',shaft_length);
fprintf('Allowable shear stress    = %.3f MPa\n',tau_allow/1e6);
fprintf('Safety factor             = %.3f\n',SafetyFactor);
fprintf('Design torque             = %.6f Nm\n',T_design);
fprintf('Required diameter         = %.3f mm\n',d_required*1e3);
fprintf('Selected diameter         = %.3f mm\n',d_selected_mm);
fprintf('Polar moment Jp           = %.6e m^4\n',Jpolar);
fprintf('Shaft stiffness Ks        = %.6f Nm/rad\n',Ks);
fprintf('Effective L               = %.6f H\n',L_effective);
fprintf('Flexible eigenfrequency   = %.6f Hz\n',f_flexible);
fprintf('Flexible period           = %.6f s\n',T_flexible);
fprintf('============================================================\n');
