## EV Drivetrain Model

This project interprets the original mechatronic system as a simplified
electric-vehicle drivetrain.

The original system contains:

- electrical resistance `R`
- electrical inductance `L`
- DC motor with motor constant `k`
- driven rotational load with polar moment of inertia `Ip`
- source voltage `Vs`
- mechanical load torque `ML`

The original project assumes a rigid shaft between the electric motor and
the driven load.

The extension investigated in this repository replaces that rigid
connection with a torsionally flexible drive shaft.

No additional motor inertia is introduced because a separate motor rotor
inertia is not specified in the original problem.

---

## Original Parameters

The parameters provided in the original problem are:

| Parameter | Value | Description |
|---|---:|---|
| `R` | 1 Ω | Electrical resistance |
| `L` | 1 H | Electrical inductance |
| `Ip` | 1 kg·m² | Polar moment of inertia of the driven roll/load |
| `k` | 10 | DC motor constant |

The original excitation cases are:

| Input | Value |
|---|---:|
| Voltage step magnitude | 1 V |
| Voltage step time | 0.1 s |
| Load torque step magnitude | 1 Nm |
| Load torque step time | 0.8 s |

---

## Electric-Vehicle Interpretation

The variables are interpreted in an EV drivetrain context as follows:

| Original Variable | EV Interpretation |
|---|---|
| `Vs` | Traction-motor supply voltage |
| `iM` | Traction-motor current |
| `omegaM` | Motor-side angular speed |
| `Ip` | Equivalent driven wheel / vehicle-side rotational inertia |
| `ML` | External road/load torque |
| `MM` | Motor electromagnetic torque |
| `vM` | Motor back-EMF |
| Flexible shaft torque | Drivetrain transmitted torque |
| Shaft twist | Torsional deformation of the drivetrain |

The model is therefore represented conceptually as

Battery / Power Supply
        |
        v
Electrical Motor Model
        |
        v
Traction Motor
        |
        v
Flexible Drive Shaft
        |
        v
Equivalent Wheel / Vehicle Load
