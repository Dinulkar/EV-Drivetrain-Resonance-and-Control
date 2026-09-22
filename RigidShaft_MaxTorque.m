%% RigidShaft_MaxTorque.m
% Determine the maximum motor torque from the ORIGINAL rigid-shaft model.
%
% Design cases:
%   1) 1 V voltage step, M_L = 0
%   2) 50%% duty pulse excitation at T/2, T, 2T and 10T, M_L = 0
%
% The resulting maximum is saved in RigidShaft_MaxTorque.mat and is used
% by FlexibleShaft_parameters.m for the shaft-diameter calculation.

run('RigidShaft_parameters.m');

opts = odeset('RelTol',RelTol,'AbsTol',AbsTol,'MaxStep',MaxStep);
x0 = [0;0];

%% 1) Voltage step
[t_step,x_step] = ode45(@(t,x) rigid_step_ode(t,x,R,L,Ip,k, ...
    Vs_step_amp,t_step_V), [0 StepStopTime], x0, opts);

i_step = x_step(:,1);
Tmotor_step = k*i_step;
[Tmax_step,idxStep] = max(abs(Tmotor_step));
t_Tmax_step = t_step(idxStep);

%% 2) Pulse periods
periods = [PulsePeriod_halfT, PulsePeriod_T, PulsePeriod_2T, PulsePeriod_10T];
periodNames = ["T/2","T","2T","10T"];

Tmax_pulse = zeros(size(periods));
t_Tmax_pulse = zeros(size(periods));
pulseResults = cell(numel(periods),1);

for n = 1:numel(periods)
    P = periods(n);

    [tp,xp] = ode45(@(t,x) rigid_pulse_ode(t,x,R,L,Ip,k, ...
        Vs_step_amp,t_step_V,P), [0 PulseStopTime], x0, opts);

    ip = xp(:,1);
    wp = xp(:,2);
    Tm = k*ip;

    [Tmax_pulse(n),idx] = max(abs(Tm));
    t_Tmax_pulse(n) = tp(idx);

    pulseResults{n}.time = tp;
    pulseResults{n}.current = ip;
    pulseResults{n}.speed = wp;
    pulseResults{n}.motorTorque = Tm;
end

allMax = [Tmax_step,Tmax_pulse];
allNames = ["Voltage step","Pulse T/2","Pulse T","Pulse 2T","Pulse 10T"];

[Tmax_rigid,iMax] = max(allMax);
Tmax_case = allNames(iMax);

if iMax == 1
    Tmax_time = t_Tmax_step;
else
    Tmax_time = t_Tmax_pulse(iMax-1);
end

save('RigidShaft_MaxTorque.mat', ...
    'Tmax_rigid','Tmax_case','Tmax_time', ...
    'Tmax_step','Tmax_pulse','t_Tmax_pulse', ...
    'T_rigid','f_rigid','periods','periodNames');

fprintf('\n============================================================\n');
fprintf('RIGID-SHAFT MAXIMUM TORQUE FOR FLEXIBLE-SHAFT DESIGN\n');
fprintf('============================================================\n');
fprintf('Voltage-step maximum torque : %.6f Nm\n',Tmax_step);
for n = 1:numel(periods)
    fprintf('Pulse %-3s maximum torque      : %.6f Nm\n', ...
        char(periodNames(n)),Tmax_pulse(n));
end
fprintf('\nOverall maximum torque       : %.6f Nm\n',Tmax_rigid);
fprintf('Occurs in                    : %s\n',char(Tmax_case));
fprintf('Saved to RigidShaft_MaxTorque.mat\n');
fprintf('============================================================\n');

% Plot resonant P=T case
r = pulseResults{2};
figure('Name','Rigid shaft - resonant pulse P=T');
plot(r.time,r.motorTorque,'LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('Motor torque [Nm]');
title('Rigid-shaft motor torque for pulse period P=T');

%% Local functions
function dx = rigid_step_ode(t,x,R,L,Ip,k,VsAmp,tStep)
    i = x(1);
    omega = x(2);

    Vs = VsAmp*(t >= tStep);
    ML = 0;

    di = (Vs - R*i - k*omega)/L;
    domega = (ML + k*i)/Ip;

    dx = [di;domega];
end

function dx = rigid_pulse_ode(t,x,R,L,Ip,k,VsAmp,tStart,P)
    i = x(1);
    omega = x(2);

    if t < tStart
        Vs = 0;
    else
        phase = mod(t-tStart,P);
        Vs = VsAmp*(phase < P/2);
    end

    ML = 0;

    di = (Vs - R*i - k*omega)/L;
    domega = (ML + k*i)/Ip;

    dx = [di;domega];
end
