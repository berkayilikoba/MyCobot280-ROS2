#include "limo_base/limo_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace AgileX
{
namespace kinematics
{

double innerToCentral(double inner, double wheelbase, double track)
{
  if (inner == 0.0) {
    return 0.0;
  }
  double sign = (inner < 0.0) ? -1.0 : 1.0;
  double abs_inner = std::fabs(inner);
  double r = wheelbase / std::tan(abs_inner) + track / 2.0;
  double central = std::atan(wheelbase / r);
  return sign * central;
}

double centralToInner(double central, double wheelbase, double track)
{
  if (central == 0.0) {
    return 0.0;
  }
  double sign = (central < 0.0) ? -1.0 : 1.0;
  double abs_c = std::fabs(central);
  double num = 2.0 * wheelbase * std::sin(abs_c);
  double denom = 2.0 * wheelbase * std::cos(abs_c) - track * std::sin(abs_c);
  double inner = std::atan(num / denom);
  return sign * inner;
}

bool twistToInnerSteering(
  double v, double w, double wheelbase, double track, double max_inner,
  double * inner_out)
{
  if (!std::isfinite(v) || !std::isfinite(w) ||
      !std::isfinite(wheelbase) || !std::isfinite(track) ||
      !std::isfinite(max_inner))
  {
    return false;
  }
  if (inner_out == nullptr) {
    return false;
  }

  if (w == 0.0) {
    *inner_out = 0.0;
    return true;
  }

  if (v == 0.0) {
    *inner_out = (w < 0.0) ? -max_inner : max_inner;
    return true;
  }

  double r = v / w;
  double sign_r = (r < 0.0) ? -1.0 : 1.0;
  if (std::fabs(r) < track / 2.0) {
    r = sign_r * (track / 2.0 + 0.01);
  }

  double central = std::atan(wheelbase / r);
  double inner = centralToInner(central, wheelbase, track);

  if (inner > max_inner) {
    inner = max_inner;
  } else if (inner < -max_inner) {
    inner = -max_inner;
  }

  *inner_out = inner;
  return true;
}

double ackermannYawRate(double v, double inner, double wheelbase, double track)
{
  if (!std::isfinite(v) || !std::isfinite(inner) ||
      !std::isfinite(wheelbase) || !std::isfinite(track))
  {
    return 0.0;
  }
  if (inner == 0.0 || v == 0.0) {
    return 0.0;
  }

  double sign_v = (v < 0.0) ? -1.0 : 1.0;
  double sign_inner = (inner < 0.0) ? -1.0 : 1.0;
  double abs_v = std::fabs(v);
  double abs_inner = std::fabs(inner);
  double r = wheelbase / std::tan(abs_inner) + track / 2.0;
  if (r == 0.0) {
    return 0.0;
  }
  double w = abs_v / r;
  return sign_v * sign_inner * w;
}

int16_t toCommand(double value)
{
  if (!std::isfinite(value)) {
    return 0;
  }
  double scaled = value * 1000.0;
  if (scaled >= 32767.0) {
    return 32767;
  }
  if (scaled <= -32768.0) {
    return -32768;
  }
  return static_cast<int16_t>(std::lround(scaled));
}

int32_t decodeInt32BE(const uint8_t * data)
{
  if (data == nullptr) {
    return 0;
  }
  uint32_t u = (static_cast<uint32_t>(data[0]) << 24) |
               (static_cast<uint32_t>(data[1]) << 16) |
               (static_cast<uint32_t>(data[2]) << 8) |
               (static_cast<uint32_t>(data[3]));
  if (u >= 0x80000000u) {
    return static_cast<int32_t>(static_cast<int64_t>(u) - 0x100000000LL);
  }
  return static_cast<int32_t>(u);
}

double yawDeltaDeg(double present_deg, double last_deg, double dead_band_deg)
{
  if (!std::isfinite(present_deg) || !std::isfinite(last_deg) ||
      !std::isfinite(dead_band_deg))
  {
    return 0.0;
  }
  double delta = present_deg - last_deg;
  delta = std::fmod(delta + 180.0, 360.0);
  if (delta < 0.0) {
    delta += 360.0;
  }
  delta -= 180.0;

  if (dead_band_deg > 0.0 && std::fabs(delta) < dead_band_deg) {
    return 0.0;
  }
  return delta;
}

}  // namespace kinematics
}  // namespace AgileX
