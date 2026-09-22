# EV-Drivetrain-Resonance-and-Control

MATLAB/Simulink simulation of an electric vehicle drivetrain with a DC traction motor, flexible drive shaft, wheel/load inertia, torsional resonance analysis, load disturbances, and closed-loop speed control.

## Overview

This project investigates the dynamic behavior of a simplified electric vehicle drivetrain.

The drivetrain consists of:

- DC traction motor
- Motor electrical dynamics
- Motor rotor inertia
- Flexible drive shaft
- Wheel / vehicle-side equivalent inertia
- External road-load torque
- Speed feedback controller

Unlike a rigid drivetrain model, the motor and wheel are allowed to rotate at different instantaneous angular velocities because the drive shaft has finite torsional stiffness and damping.

This makes it possible to investigate:

- motor current response
- motor and wheel speed oscillations
- drivetrain torsional vibration
- shaft torque
- shaft twist angle
- resonance frequencies
- response to road-load disturbances
- periodic excitation
- closed-loop speed regulation

---

## EV Drivetrain Representation

The original mechanical system is interpreted as an electric vehicle drivetrain:

| Model Variable | EV Interpretation |
|---|---|
| `Vs` | Battery / inverter output voltage |
| `iM` | Traction motor current |
| `omega_M` | Motor angular speed |
| `omega_R` | Wheel / axle angular speed |
| `MM` | Motor electromagnetic torque |
| `ML` | Road / vehicle load torque |
| `Jm` | Motor rotor inertia |
| `Jr` | Wheel and reflected vehicle inertia |
| `Ks` | Drivetrain torsional stiffness |
| `Cs` | Drivetrain torsional damping |
| `DeltaTheta` | Drivetrain shaft twist angle |
| `ShaftTorque` | Torque transmitted through the drivetrain |

The simplified drivetrain structure is

```text
Battery / Inverter
        |
        v
 Traction Motor
        |
        v
 Flexible Shaft
        |
        v
 Wheel / Vehicle Load
