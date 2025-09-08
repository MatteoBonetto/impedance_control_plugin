# impedance_control plugin for MADS

This is a Filter plugin for [MADS](https://github.com/MADS-NET/MADS).

The `impedance_control` plugin implements impedance-based control logic for mobile robots (walkers). It computes the forces and torques to be applied to the wheels based on physical parameters, ego state signals, and configurable control strategies. The plugin supports advanced features such as incipit logic for starting movement and dynamic adjustment of control parameters via MQTT or INI configuration.

*Required MADS version: 1.3.1.*

## Supported platforms

Currently, the supported platforms are:

* **Linux**
* **MacOS**
* **Windows**

## Installation

Linux and MacOS:

```bash
cmake -Bbuild -DCMAKE_INSTALL_PREFIX="$(mads -p)"
cmake --build build -j4
sudo cmake --install build
```

Windows:

```powershell
cmake -Bbuild -DCMAKE_INSTALL_PREFIX="$(mads -p)"
cmake --build build --config Release
cmake --install build --config Release
```

## INI settings

The plugin supports the following settings in the INI file:

```ini
[impedance_control]
# impedance parameters
M_v = <double>      # Mass for linear motion
C_v = <double>      # Damping for linear motion
K_v = <double>      # Stiffness for linear motion
M_w = <double>      # Mass for angular motion
C_w = <double>      # Damping for angular motion
K_w = <double>      # Stiffness for angular motion

# incipit parameters
incipit_function = <string>         # Function for starting movement (none_incipit, decrease_linearly_with_velocity, decrease_exponentialy_with_time)
v_considered_as_moving = <double>   # Speed threshold to consider as moving
incipit_parameter = <double>        # Parameter for incipit function
F0_max = <double>                   # Maximum force applied when not moving
g = <double>                        # Coefficient for force increase when not moving

# delta_theta function
delta_theta_function = <string>     # Function for angle error (none, atan, sqrt)
```

All settings are optional; if omitted, the default values are used.

---

# ImpedanceControlManager Class Documentation

## Overview

`ImpedanceControlManager` is a C++ class that implements impedance-based control for mobile robots. It calculates the forces and torques to be applied to the wheels based on the robot's physical parameters, ego state signals, and configurable control strategies. The class also manages incipit logic for assisting the robot in starting movement.

---

## Class Declaration

```cpp
class ImpedanceControlManager {
public:
  ImpedanceControlManager(walker_phisical_parameter walker_params);
  ~ImpedanceControlManager() = default;

  return_type compute_incipit();
  return_type compute_impedance_control();

  std::array<double, Forces_Measured_COUNT> forces;
  double right_wheel_torque() const;
  double left_wheel_torque() const;

  incipit_parameters incipit_param;
  walker_phisical_parameter walker_params;
  impedance_parameters impedance_params;
  delta_theta_function delta_theta_f;
  std::map<std::string, Signal> ego_state_map;
  double time_interval;
};
```

---

## Public Methods

### `ImpedanceControlManager(walker_phisical_parameter walker_params)`
Constructor. Initializes the manager with the robot's physical parameters.

### `return_type compute_incipit()`
Computes the incipit force offset to assist the robot in starting movement, based on current velocity and configured incipit strategy.

### `return_type compute_impedance_control()`
Calculates the wheel torques using impedance control equations, ego state signals, and current incipit force.

### `double right_wheel_torque() const`
Returns the computed torque for the right wheel.

### `double left_wheel_torque() const`
Returns the computed torque for the left wheel.

---

## Public Structs

### `impedance_parameters`
Holds impedance control parameters:

| Member Name | Type    | Description                  |
|-------------|---------|------------------------------|
| `M_v`       | double  | Mass for linear motion       |
| `C_v`       | double  | Damping for linear motion    |
| `K_v`       | double  | Stiffness for linear motion  |
| `M_w`       | double  | Mass for angular motion      |
| `C_w`       | double  | Damping for angular motion   |
| `K_w`       | double  | Stiffness for angular motion |

### `incipit_parameters`
Holds incipit logic parameters:

| Member Name              | Type    | Description                                  |
|--------------------------|---------|----------------------------------------------|
| `g`                      | double  | Coefficient for force increase when not moving|
| `v_considered_as_moving` | double  | Speed threshold to consider as moving        |
| `incipit_parameter`      | double  | Parameter for incipit function               |
| `F0_max`                 | double  | Maximum force applied when not moving        |
| `incipit_function`       | enum    | Incipit function type                        |

---

## Enums

- `incipit_function_start_moving`: Selects the incipit strategy (`none_incipit`, `decrease_linearly_with_velocity`, `decrease_exponentialy_with_time`).
- `delta_theta_function`: Selects the transformation for angle error (`none`, `atan_function`, `sqrt_function`).

---

## Usage Notes

- **Extensibility:** You can modify the incipit and impedance control logic by extending the respective methods.
- **Integration:** The class is designed to be used within the MADS plugin framework and can be configured via INI or MQTT.

---

## Example Implementation

```cpp
ImpedanceControlManager manager(walker_params);
// Update ego_state_map and time_interval before calling
manager.compute_incipit();
manager.compute_impedance_control();
double left_torque = manager.left_wheel_torque();
double right_torque = manager.right_wheel_torque();
```

---

## Summary

`ImpedanceControlManager` provides a modular and extensible foundation for implementing impedance-based control in mobile robots. It supports advanced features such as incipit logic for movement initiation and dynamic parameter adjustment, making it suitable for research and real-world applications in assistive robotics.

---

## Executable demo

When the test executable is run, it loads parameters and input data from JSON files, initializes the plugin, and repeatedly processes the input to compute and print the output wheel torques. This allows you to test the impedance control logic with sample data and verify correct operation.

---