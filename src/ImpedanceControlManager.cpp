#include "ImpedanceControlManager.h"

ImpedanceControlManager::ImpedanceControlManager(
    walker_phisical_parameter walker_params)
    : walker_params(walker_params) {
  common_walker::init();
  common_walker::init_ego_state_map(ego_state_map);

  std::cerr << "Walker parameters: " << walker_params.wheel_radius << ", "
            << walker_params.wheel_base << ", " << walker_params.walker_mass
            << ", " << walker_params.wheel_mass << ", " << walker_params.I_wheel
            << ", " << walker_params.I_body << std::endl;
  if (walker_params.wheel_radius == 0.0 || walker_params.wheel_base == 0.0 ||
      walker_params.walker_mass == 0.0 || walker_params.wheel_mass == 0.0 ||
      walker_params.I_wheel == 0.0 || walker_params.I_body == 0.0) {
    throw std::invalid_argument("Walker parameters must be non-zero.");
  }
}

return_type ImpedanceControlManager::compute_incipit() {

  // Implementation of the incipit function logic
  // incipit applies a force offset in case velocities are low
  _incipit = 0;
  const double speed_incipit = incipit_param.v_considered_as_moving;
  _is_moving = ego_state_map["speed"] > speed_incipit;
  const double exponent = incipit_param.incipit_parameter;

  if ((exponent <= 0 || speed_incipit < 0 || incipit_param.F0_max < 0 ||
       incipit_param.g <= 0) &&
      (incipit_param.incipit_function != none_incipit)) {
    std::cerr << "The sign of one or more incipit parameters is incorrect."
              << std::endl;
    return return_type::critical;
  }

  if (_is_moving) {
    _time_no_move = 0;

    if (_incipit > 0) {
      switch (incipit_param.incipit_function) {
      case none_incipit: {
        _incipit = 0;
        break;
      }
      case decrease_linearly_with_velocity: {
        // in case it moves a force decrease linearly as velocity increase
        // to push walker
        _incipit = -_F0_start_moving / speed_incipit *
                       std::abs(ego_state_map["speed"]) +
                   _F0_start_moving;
        break;
      }
      case decrease_exponentialy_with_time: {
        _time_moving += time_interval;
        _incipit = _F0_start_moving * std::exp(-exponent * _time_moving);
        break;
      }
      default: {
        std::cerr << "incipit function is not among the available "
                     "options: "
                  << incipit_param.incipit_function << std::endl;
        return return_type::critical;
        break;
      }
      }
    } else {
      _incipit = 0;
    }

  } else if (!_is_moving && incipit_param.incipit_function != none_incipit) {
    // in case it does not move a force increase linearly in time to push walker
    // untill it reaches F0_max
    if (_F0_start_moving < incipit_param.F0_max) {
      _time_no_move += time_interval;
      _incipit = incipit_param.g * _time_no_move;
    } else { // _F0_start_moving < _F0_max -> false
      _incipit = incipit_param.F0_max;
    }

    _F0_start_moving = _incipit;
    _time_moving = 0;
  }

  return return_type::success;
}

return_type ImpedanceControlManager::compute_impedance_control() {
  // check whether parameters are sent
  if (impedance_params.M_v == 0 || impedance_params.M_w == 0) {
    std::cerr << "Impedance parameters not set correctly." << std::endl;
    std::cerr << "M_v: " << impedance_params.M_v
              << ", M_w: " << impedance_params.M_w << std::endl;
    return return_type::retry;
  }

  // ************************* Compute the accelerations
  // *************************
  double acc_d = (ego_state_map["F_x"] + _incipit -
                  impedance_params.C_v * ego_state_map["speed"] -
                  impedance_params.K_v * (ego_state_map["space_linear"] -
                                          ego_state_map["k_delta_ref"])) /
                 impedance_params.M_v;

  // compute the angle between the current angle and the reference angle with
  // versors v1 x v2 = sin(delta)
  double cross_prod =
      std::cos(ego_state_map["theta"]) *
          std::sin(ego_state_map["k_theta_ref"]) -
      std::cos(ego_state_map["k_theta_ref"]) * std::sin(ego_state_map["theta"]);
  double delta_theta = std::asin(cross_prod);

  switch (delta_theta_f) {
  case none:
    // No transformation applied
    break;
  case atan_function:
    delta_theta = std::atan(delta_theta / (M_PI - std::abs(delta_theta)));
    break;
  case sqrt_function:
    if (delta_theta > 0) {
      delta_theta = std::sqrt(delta_theta);
    } else {
      delta_theta = -std::sqrt(-delta_theta);
    }
    break;
  }

  double angular_acc_d =
      (ego_state_map["T"] -
       impedance_params.C_w * ego_state_map["angular_speed"] -
       impedance_params.K_w * delta_theta) /
      impedance_params.M_w;

  // ************************* Compute torques *************************
  double external_compensation_right = 0, acc_factor_right = 0,
         angular_acc_factor_right = 0;
  double external_compensation_left = 0, acc_factor_left = 0,
         angular_acc_factor_left = 0;

  // in case we are artificially adding a value of Force, we have to subtract
  // the value of compensation of this force by external_compensation_right and
  // external_compensation_left
  external_compensation_left =
      -walker_params.wheel_radius / 2 *
      ((ego_state_map["F_x"] - _incipit) -
       2.0 * ego_state_map["T"] / walker_params.wheel_base);
  acc_factor_left =
      (walker_params.I_wheel / walker_params.wheel_radius +
       1.0 / 2.0 *
           (walker_params.walker_mass + 2.0 * walker_params.wheel_mass) *
           walker_params.wheel_radius);
  angular_acc_factor_left =
      -walker_params.I_body * walker_params.wheel_radius /
          walker_params.wheel_base -
      (walker_params.I_wheel + walker_params.wheel_mass *
                                   walker_params.wheel_radius *
                                   walker_params.wheel_radius) *
          walker_params.wheel_base / (2 * walker_params.wheel_radius);

  // in case we are artificially adding a value of Force, we have to subtract
  // the value of compensation of this force by external_compensation_right and
  // external_compensation_left
  external_compensation_right =
      -walker_params.wheel_radius / 2 *
      ((ego_state_map["F_x"] - _incipit) +
       2 * ego_state_map["T"] / walker_params.wheel_base);
  ;
  acc_factor_right = acc_factor_left;
  angular_acc_factor_right = -angular_acc_factor_left;

  // in case there is a high incipit and it is moving -> user is contrasting the
  // movement -> avoid to rotate
  if ((std::abs(_incipit) > 15) && (!_is_moving)) {
    angular_acc_factor_right = 0;
    angular_acc_factor_left = 0;
  }

  _right_wheel_torque = external_compensation_right + acc_factor_right * acc_d +
                        angular_acc_factor_right * angular_acc_d;
  _left_wheel_torque = external_compensation_left + acc_factor_left * acc_d +
                       angular_acc_factor_left * angular_acc_d;

  _right_wheel_torque = _right_wheel_torque * 100;
  _left_wheel_torque = _left_wheel_torque * 100;

  return return_type::success;
}
