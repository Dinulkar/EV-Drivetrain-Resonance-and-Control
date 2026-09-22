%% RigidShaft_parameters.m
% EV-Drivetrain-Resonance-and-Control
% Original rigid-shaft plant parameters from the supplied problem.
%
% Original sign convention:
%   M_M = -k*i_M
%   domega/dt = (M_L - M_M)/I_p
% Therefore:
%   I_p*domega/dt = M_L + k*i_M
%
% If M_L is interpreted later as a positive resisting road-load magnitude,
% use M_L = -T_road in this source-coordinate model.

clearvars;
clc;

%% Original plant parameters
R  = 1;          % Armature resistance [Ohm]
L  = 1;          % Armature inductance [H]
Ip = 1;          % Roll/load polar moment of inertia [kg*m^2]
k  = 10;         % Motor constant [Nm/A] and [V/(rad/s)]

%% Original excitation data
Vs_step_amp = 1;          % Voltage step amplitude [V]
t_step_V    = 0.1;        % Voltage step time [s]

ML_disturbance_amp = 1;   % Signed load-moment step [Nm]
t_step_M           = 0.8; % Load-moment step time [s]

% Default Simulink setting = voltage-step task with M_L = 0
ML_model_amp = 0;

%% Rigid-shaft state-space model
% States: x = [i_M; omega]
% Inputs: u = [V_s; M_L]
%
% di/dt     = -(R/L)i - (k/L)omega + (1/L)V_s
% domega/dt =  (k/Ip)i + (1/Ip)M_L

A_rigid = [-R/L, -k/L;
            k/Ip,  0];

B_rigid = [1/L, 0;
           0,   1/Ip];

C_rigid = eye(2);
D_rigid = zeros(2,2);

lambda_rigid = eig(A_rigid);

idx = find(imag(lambda_rigid) > 0, 1);
omega_d_rigid = imag(lambda_rigid(idx));
f_rigid = omega_d_rigid/(2*pi);
T_rigid = 1/f_rigid;

%% Pulse periods required by the original assignment
PulsePeriod_halfT = T_rigid/2;
PulsePeriod_T     = T_rigid;
PulsePeriod_2T    = 2*T_rigid;
PulsePeriod_10T   = 10*T_rigid;

%% Simulation settings
StepStopTime  = 5;       % [s]
PulseStopTime = 30;      % [s]
MaxStep       = 1e-3;    % [s]
RelTol        = 1e-9;
AbsTol        = 1e-11;

fprintf('\n============================================================\n');
fprintf('RIGID-SHAFT PARAMETERS\n');
fprintf('============================================================\n');
fprintf('R  = %.6g Ohm\n', R);
fprintf('L  = %.6g H\n', L);
fprintf('Ip = %.6g kg*m^2\n', Ip);
fprintf('k  = %.6g\n', k);
fprintf('Voltage step = %.6g V at t = %.6g s\n', Vs_step_amp, t_step_V);
fprintf('Rigid oscillation frequency = %.6f Hz\n', f_rigid);
fprintf('Rigid oscillation period    = %.6f s\n', T_rigid);
fprintf('============================================================\n');
