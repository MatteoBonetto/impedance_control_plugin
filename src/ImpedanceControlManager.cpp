#include "ImpedanceControlManager.h"

ImpedanceControlManager::ImpedanceControlManager(double wheel_radius, double wheel_base, double g) {
  _wheel_radius = wheel_radius;
  _wheel_base = wheel_base;
  _g = g;
  if (wheel_radius == 0.0 || wheel_base == 0.0 || g == 0.0) {
    throw std::invalid_argument(
        "wheel_radius, and wheel_base must be non-zero.");
  }
}

return_type ImpedanceControlManager::compute_incipit() {

  // Implementation of the incipit function logic
  // incipit applies a force offset in case velocities are low
  _incipit = 0;
  const double speed_incipit = _incipit_parameters._v_considered_as_moving;
  bool is_moving = ego_state.speed > speed_incipit;
  const double exponent = _incipit_parameters._incipit_parameter;
  
  if (exponent <= 0 || speed_incipit < 0 || _incipit_parameters._F0_max < 0) {
    std::cerr << "The sign of one or more incipit parameters is incorrect." << std::endl;
    return return_type::critical;
  }

  if (is_moving) {
    _time_no_move = 0;

    if(_incipit > 0) {
      switch (_incipit_parameters.incipit_function) {
      case none: {
        _incipit = 0;
        break;
      }
      case decrease_linearly_with_velocity: {
        // in case it moves a force decrease linearly as velocity increase
        // to push walker
        _incipit = -_F0_start_moving / speed_incipit * std::abs(ego_state.speed) +
            _F0_start_moving;
        break;
      }
      case decrease_exponentialy_with_time: {
        _time_moving += ego_state.time_interval;
        _incipit = _F0_start_moving * std::exp(-exponent * _time_moving);
        break;
      }
      default: {
        std::cerr << "incipit function is not among the available "
                    "options: "
                  << _incipit_parameters.incipit_function << std::endl;
        return return_type::critical;
        break;
      }
      }
    }else {
      _incipit = 0;
    }

  } else if(!is_moving && _incipit_parameters.incipit_function != none) {
    // in case it does not move a force increase linearly in time to push walker
    // untill it reaches F0_max
    if (_F0_start_moving < _incipit_parameters._F0_max) {
      _time_no_move += ego_state.time_interval;
      _incipit = _g * _time_no_move;
    } else { // _F0_start_moving < _F0_max -> false
      _incipit = _incipit_parameters._F0_max;
    }

    _F0_start_moving = _incipit;
    _time_moving = 0;
  }

  return return_type::success;
}

return_type ImpedanceControlManager::compute_impedance_control() {
  double F_x = _incipit;
  double _acc_d = (F_x - _impedance_params.C_v * ego_state.speed - _impedance_params.K_v * (ego_state.space_linear- ego_state.k_delta_ref)) / _impedance_params.M_v;

  _right_wheel_torque = _incipit * 100;
  _left_wheel_torque = _incipit * 100;

  return return_type::success;

}

