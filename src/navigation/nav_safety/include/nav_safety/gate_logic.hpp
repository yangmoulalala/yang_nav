// Pure decision logic of cmd_gate (no ROS), unit-tested in test/test_gate_logic.cpp.
#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>

namespace nav_safety
{

// Navigation mode published by ZoneAwareController on /nav_mode. Recovery is a reserved fallback.
enum class Mode { Normal = 0, Tunnel = 1, Ramp = 2, Recovery = 3 };
constexpr std::size_t kModeCount = 4;

inline std::optional<Mode> parse_mode(const std::string & s)
{
  if (s == "normal") {return Mode::Normal;}
  if (s == "tunnel") {return Mode::Tunnel;}
  if (s == "ramp") {return Mode::Ramp;}
  if (s == "recovery") {return Mode::Recovery;}
  return std::nullopt;
}

inline const char * mode_name(Mode m)
{
  static const char * names[] = {"normal", "tunnel", "ramp", "recovery"};
  return names[static_cast<int>(m)];
}

enum class Localization { Unavailable, Trusted, Extrapolated };

inline std::string_view state_field(std::string_view text, std::string_view key)
{
  const auto trim = [](std::string_view value) {
      const auto first = value.find_first_not_of(" \t");
      return first == std::string_view::npos ? std::string_view{} :
             value.substr(first, value.find_last_not_of(" \t") - first + 1);
    };
  while (!text.empty()) {
    const auto end = text.find(';');
    const auto part = text.substr(0, end);
    const auto eq = part.find('=');
    if (eq != std::string_view::npos && trim(part.substr(0, eq)) == key) {
      return trim(part.substr(eq + 1));
    }
    if (end == std::string_view::npos) {break;}
    text.remove_prefix(end + 1);
  }
  return {};
}

inline Localization parse_localization(std::string_view text)
{
  auto trusted = std::string(state_field(text, "pose_trusted"));
  std::transform(trusted.begin(), trusted.end(), trusted.begin(),
    [](unsigned char ch) {return std::tolower(ch);});
  if (trusted == "true") {return Localization::Trusted;}
  if (state_field(text, "status") == "SUSPECT" &&
    state_field(text, "pose_extrapolated") == "true")
  {
    return Localization::Extrapolated;
  }
  return Localization::Unavailable;
}

struct Limits
{
  double max_linear;   // |(vx, vy)|, m/s
  double max_angular;  // |wz|, rad/s
};

constexpr Limits kExtrapolatedLimits{0.6, 1.0};

struct GateConfig
{
  double input_timeout = 0.25;
  double state_timeout = 1.0;
  bool require_trusted = true;
  bool require_observation = false;
  double observation_timeout = 0.5;
  std::array<Limits, kModeCount> limits{{
    {1.6, 2.5},   // normal
    {0.8, 1.0},   // tunnel: backstop above zone.tunnel_speed + lateral correction
    {2.0, 2.5},   // ramp: the zone tracker supplies the motion profile
    {0.6, 1.5},   // recovery
  }};
};

struct GateInput
{
  bool have_cmd = false;
  double cmd_age = 0.0;
  bool have_state = false;
  double state_age = 0.0;
  Localization localization = Localization::Unavailable;
  bool have_observation = false;
  double observation_age = 0.0;
};

enum class Decision
{
  Idle,  // no fresh input: stop briefly, then leave the output topic alone
  Hold,  // fresh input but not allowed to move: publish zero
  Pass,  // forward the (limited) command
};

inline Decision decide(const GateConfig & c, const GateInput & in, std::string & reason)
{
  if (!in.have_cmd || !std::isfinite(in.cmd_age) || in.cmd_age < 0.0 || in.cmd_age > c.input_timeout) {
    reason = "no input";
    return Decision::Idle;
  }
  if (c.require_trusted) {
    if (!in.have_state) {reason = "no localization state"; return Decision::Hold;}
    if (!std::isfinite(in.state_age) || in.state_age < 0.0 || in.state_age > c.state_timeout) {
      reason = "localization state stale"; return Decision::Hold;
    }
    if (in.localization == Localization::Unavailable) {
      reason = "localization not trusted";
      return Decision::Hold;
    }
  }
  if (c.require_observation && (!in.have_observation || !std::isfinite(in.observation_age) ||
    in.observation_age < 0.0 || in.observation_age > c.observation_timeout))
  {
    reason = "obstacle observation stale";
    return Decision::Hold;
  }
  reason.clear();
  return Decision::Pass;
}

inline Limits motion_limits(Limits limits, double speed_limit, Localization localization)
{
  limits.max_linear = std::min(limits.max_linear, speed_limit);
  if (localization == Localization::Extrapolated) {
    limits.max_linear = std::min(limits.max_linear, kExtrapolatedLimits.max_linear);
    limits.max_angular = std::min(limits.max_angular, kExtrapolatedLimits.max_angular);
  }
  return limits;
}

struct Cmd
{
  double vx = 0.0, vy = 0.0, wz = 0.0;
};

// Scale (vx, vy) together so the direction is kept; clamp wz.
inline Cmd limit(Cmd c, const Limits & l)
{
  if (!std::isfinite(c.vx) || !std::isfinite(c.vy) || !std::isfinite(c.wz)) {
    return {};
  }
  const double v = std::hypot(c.vx, c.vy);
  if (v > l.max_linear && v > 0.0) {
    const double s = l.max_linear / v;
    c.vx *= s;
    c.vy *= s;
  }
  c.wz = std::clamp(c.wz, -l.max_angular, l.max_angular);
  return c;
}

}  // namespace nav_safety
