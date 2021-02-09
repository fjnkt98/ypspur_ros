#ifndef YPSPUR_ROS2_SPEED_LIMITER_HPP_
#define YPSPUR_ROS2_SPEED_LIMITER_HPP_

#include <algorithm>

namespace ypspur_ros {
  class SpeedLimiter {
    public:
      SpeedLimiter(bool has_velocity_limitation = false,
                   double v_min = 0.0,
                   double v_max = 0.0,
                   bool has_acceleration_limitation = false,
                   double a_min = 0.0,
                   double a_max = 0.0)
      :has_velocity_limitation_(has_velocity_limitation),
       v_min_(v_min),
       v_max_(v_max),
       has_acceleration_limitation_(has_acceleration_limitation),
       a_min_(a_min),
       a_max_(a_max)
      {
      }

      double apply_limitation(const double v_curr, const double v_last, const double dt) const {
        double tmp = v_curr;

        tmp = limit_acceleration(tmp, v_last, dt);
        tmp = limit_velocity(tmp);

        return tmp;
      }

      double limit_acceleration(const double v_curr, const double v_last, const double dt) const {
        if (has_acceleration_limitation_ == true) {
          const double dv_min = a_min_ * dt;
          const double dv_max = a_max_ * dt;

          const double dv = std::min(std::max((v_curr - v_last), dv_min), dv_max);

          return (v_last + dv);
        }

        return v_curr;
      }

      double limit_velocity(const double v_curr) const {
        if (has_velocity_limitation_ == true)
          return std::min(std::max(v_curr, v_min_), v_max_);

        return v_curr;
      }

    private:
      bool has_velocity_limitation_;
      double v_min_;
      double v_max_;
      bool has_acceleration_limitation_;
      double a_min_;
      double a_max_;
  };
} // namespace ypspur_ros

#endif
