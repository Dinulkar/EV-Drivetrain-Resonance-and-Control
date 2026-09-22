EV-Drivetrain-Resonance-and-Control
=========================================

RUN ORDER
---------

1) Rigid shaft:
       RigidShaft_parameters
       build_RigidShaft_Simulink

   This creates:
       EV_RigidShaft.slx

2) Determine the rigid-model maximum torque:
       RigidShaft_MaxTorque

   This creates:
       RigidShaft_MaxTorque.mat

3) Flexible shaft:
       FlexibleShaft_parameters
       build_FlexibleShaft_Simulink

   This creates:
       EV_FlexibleShaft.slx

4) Report source:
       EV_Drivetrain_Rigid_and_Flexible_Report.tex


MODELING NOTE
-------------

No separate motor inertia Jm is supplied by the original problem, so no
numerical Jm is invented.

The flexible shaft preserves the original modeling level:

    T_s = T_M = k*i_M
    T_s = K_s*(theta_M-theta_R)

which gives:

    DeltaTheta = k*i_M/K_s
    omega_M - omega_R = (k/K_s)*di_M/dt

and:

    (L + k^2/K_s)*di_M/dt = V_s - R*i_M - k*omega_R

The new shaft material, length, allowable stress and safety factor are
explicit design choices required for the added shaft, not original plant
parameters.
