#ifndef IMPEDANCECONTROLMANAGER_H
#define IMPEDANCECONTROLMANAGER_H

#include <cmath>  // oppure <math.h>, ma <cmath> è preferito in C++
#include <iostream>
#include <common.hpp>



/*******************************************************
 * File:           ImpedanceControlManager.h
 * Purpose:        Declaration of the ImpedanceControlManager class
 *                 responsible for ...
 *
 * Author:         Matteo Bonetto
 * Created:        22/07/2025
 *
 *******************************************************/

 enum modes {
  unknown_mode = 0,
  impedance_control_mode,
  joystick_control_mode,
  emrgency_control_mode,
  set_reference_mode,
  unsupported_mode
};

struct ego_state_estimation {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double speed = 0.0;
  double angular_speed = 0.0;
  double acceleration = 0.0;
  double angular_acceleration = 0.0;
  double k_theta_ref = 0.0;
  double k_delta_ref = 0.0;
  double space_linear = 0.0; // Linear space covered
  double time_interval = -1;
};

struct impedance_parameters {
    double M_v = 0;
    double C_v = 0; 
    double K_v = 0; 
    double M_w = 0;
    double C_w = 0; 
    double K_w = 0; 
};

enum incipit_function_start_moving {
  none = 0,
  // in case it move slowly a force decrease linearly as velocity increase to
  // push walker
  decrease_linearly_with_velocity,
  decrease_exponentialy_with_time
};

struct incipit_parameters {
  double _v_considered_as_moving = 0; // Speed threshold to consider as moving
  double _incipit_parameter = 0;      // Parameter for incipit functions
  double _F0_max = 0;         // Maximum force applied when not moving
  incipit_function_start_moving incipit_function = none;
};

class ImpedanceControlManager {
  public:
    // Constructor to initialize the manager with control parameters
    ImpedanceControlManager(double wheel_radius, double wheel_base, double g);
    ~ImpedanceControlManager() = default;
    
    return_type compute_incipit();
    void set_incipit_parameters(const incipit_parameters &params) {
      _incipit_parameters = params;
    }
    return_type compute_impedance_control();
    ego_state_estimation ego_state;
    double right_wheel_torque() const { return _right_wheel_torque; }
    double left_wheel_torque() const { return _left_wheel_torque; }
  
  private:
    // Phisical parameters
    double _wheel_radius = 0.0;
    double _wheel_base = 0.0;
    double _walker_mass = 0.0; // Mass of the walker
    double _wheel_mass = 0.0; // Mass of the wheel
    double _I_wheel = 0.0; // Moment of inertia of the wheel
    double _I_body = 0.0; // Moment of inertia of the walker except for wheels
    
    // Incipit parameters
    double _incipit = 0; // Force offset applied when moving slowly
    double _g = 0;            // Coefficient for force increase when not moving
    double _time_no_move = 0; // Time when not moving
    double _time_moving = 0;  // Time when moving
    double _F0_start_moving = 0; // Initial force when starting to move
    incipit_parameters _incipit_parameters;

    // Impedance control
    impedance_parameters _impedance_params;
    double _right_wheel_torque = 0.0; // Torque applied on the right wheel
    double _left_wheel_torque = 0.0;

};

#endif