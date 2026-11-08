#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace task_flight {
constexpr double pi = 3.14159265358979323846;
// Protocol compatibility with dynamixel_fix.cpp at 651beba: do not change 3.14.
inline uint16_t legacy_tick(double radians) {
  if (!std::isfinite(radians) || radians < -3.14 || radians > 3.14)
    throw std::invalid_argument("invalid legacy servo angle");
  return static_cast<uint16_t>(((3.14 + radians) / (2.0 * 3.14)) * 4096);
}
inline float legacy_angle(uint32_t raw) {
  return (static_cast<float>(static_cast<uint16_t>(raw)) / 4096.0f) * (2.0f * 3.14) - 3.14;
}
inline std::array<uint32_t, 4> legacy_commands(const std::vector<float>& data) {
  if (data.size() != 5) throw std::invalid_argument("servo_angle_cmd requires five slots");
  std::array<uint32_t, 4> result{};
  for (size_t i=0;i<4;++i) result[i] = legacy_tick(data[i]);
  // data[4] remains the legacy payload slot; never mapped to J1.
  return result;
}
inline std::vector<float> legacy_feedback(const std::array<uint32_t, 4>& raw) {
  return {legacy_angle(raw[0]),legacy_angle(raw[1]),legacy_angle(raw[2]),legacy_angle(raw[3]),legacy_angle(0)};
}
struct Calibration {
  int zero_tick=2048, direction=1, ticks_per_rev=4096, raw_min=0, raw_max=4095;
  double angle_min=-5*pi/180, angle_max=35*pi/180;
  bool confirmed=false;
  void validate() const {
    if ((direction!=1 && direction!=-1) || ticks_per_rev<=0 || raw_min<0 || raw_max<=raw_min
        || zero_tick<raw_min || zero_tick>raw_max || !std::isfinite(angle_min)
        || !std::isfinite(angle_max) || angle_max<=angle_min)
      throw std::invalid_argument("invalid J1 calibration/limits");
    for (double angle : {angle_min,angle_max}) {
      const double raw=zero_tick+direction*angle*ticks_per_rev/(2*pi);
      if (raw<raw_min || raw>raw_max) throw std::invalid_argument("physical limits exceed raw limits");
    }
  }
  int raw(double theta) const {
    validate();
    if (!confirmed || !std::isfinite(theta) || theta<angle_min || theta>angle_max)
      throw std::invalid_argument("uncalibrated or invalid J1 command");
    const auto result=std::lround(zero_tick+direction*theta*ticks_per_rev/(2*pi));
    if (result<raw_min || result>raw_max) throw std::invalid_argument("J1 raw target outside limits");
    return static_cast<int>(result);
  }
  double angle(int raw_tick) const {
    validate();
    if (raw_tick<raw_min || raw_tick>raw_max) throw std::invalid_argument("J1 raw feedback outside limits");
    const double theta=direction*(raw_tick-zero_tick)*(2*pi)/ticks_per_rev;
    const double half_tick=pi/ticks_per_rev;
    if (theta<angle_min-half_tick || theta>angle_max+half_tick)
      throw std::invalid_argument("J1 physical feedback outside limits");
    return theta;
  }
};
}  // namespace task_flight
