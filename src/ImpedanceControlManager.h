#ifndef IMPEDANCECONTROLMANAGER_H
#define IMPEDANCECONTROLMANAGER_H

#include <array>
#include <cmath> // oppure <math.h>, ma <cmath> è preferito in C++
#include <common.hpp>
#include <common_walker.h>
#include <iostream>
#include <tuple>

using namespace common_walker;

/*******************************************************
 * File:           ImpedanceControlManager.h
 * Purpose:        Declaration of the ImpedanceControlManager class,
 *                 responsible for managing impedance control logic
 *                 for the walker. This class computes the forces and
 *                 torques to be applied to the wheels based on physical
 *                 parameters, ego state signals, and configurable control
 *                 strategies, including incipit logic for starting movement.
 *
 * Author:         Matteo Bonetto
 * Created:        22/07/2025
 *
 *******************************************************/

struct impedance_parameters {
  double M_v = 0;
  double C_v = 0;
  double K_v = 0;
  double M_w = 0;
  double C_w = 0;
  double K_w = 0;
};

enum incipit_function_start_moving {
  none_incipit = 0,
  // in case it move slowly a force decrease linearly as velocity increase to
  // push walker
  decrease_linearly_with_velocity,
  decrease_exponentialy_with_time
};

enum delta_theta_function { none = 0, sqrt_function, atan_function };

struct incipit_parameters {
  double g = 0; // Coefficient for force increase when not moving
  double v_considered_as_moving = 0; // Speed threshold to consider as moving
  double incipit_parameter = 0;      // Parameter for incipit functions
  double F0_max = 0;                 // Maximum force applied when not moving
  incipit_function_start_moving incipit_function = none_incipit;
};

class ImpedanceControlManager {
public:
  // Constructor to initialize the manager with control parameters
  ImpedanceControlManager(walker_phisical_parameter walker_params);
  ~ImpedanceControlManager() = default;

  return_type compute_incipit();
  return_type compute_impedance_control();

  std::array<double, Forces_Measured_COUNT> forces;
  double right_wheel_torque() const { return _right_wheel_torque; }
  double left_wheel_torque() const { return _left_wheel_torque; }

  // Incipit parameters
  incipit_parameters incipit_param;
  // Phisical parameters
  walker_phisical_parameter walker_params;
  // Impedance control
  impedance_parameters impedance_params;
  delta_theta_function delta_theta_f = none;
  // Signals
  std::map<std::string, Signal> ego_state_map;
  double time_interval = 0.0;

private:
  // Incipit variables
  double _incipit = 0;         // Force offset applied when moving slowly
  double _time_no_move = 0;    // Time when not moving
  double _time_moving = 0;     // Time when moving
  double _F0_start_moving = 0; // Initial force when starting to move
  bool _is_moving = false;

  // Impedance control
  double _right_wheel_torque = 0.0; // Torque applied on the right wheel
  double _left_wheel_torque = 0.0;
};

#endif