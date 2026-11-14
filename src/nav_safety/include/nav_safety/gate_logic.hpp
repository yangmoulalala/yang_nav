// Pure decision logic of cmd_gate (no ROS), unit-tested in test/test_gate_logic.cpp.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>

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

// Backend state string 'state=NORMAL;pose_trusted=true;...' -> pose_trusted.
inline bool parse_trusted(const std::string & text)
{
  std::size_t start = 0;
  while (start <= text.size()) {
    std::size_t end = text.find(';', start);
    if (end == std::string::npos) {end = text.size();}
    const std::string part = text.substr(start, end - start);
    const std::size_t eq = part.find('=');
    if (eq != std::string::npos) {
      auto trim = [](std::string s) {
          const auto b = s.find_first_not_of(" \t");
          const auto e = s.find_last_not_of(" \t");
          return b == std::string::npos ? std::string() : s.substr(b, e - b + 1);
        };
      if (trim(part.substr(0, eq)) == "pose_trusted") {
        std::string v = trim(part.substr(eq + 1));
        std::transform(v.begin(), v.end(), v.begin(), ::tolower);
        return v == "true";
      }
    }
    start = end + 1;
  }
  return false;
}

struct Limits
{
  double max_linear;   // |(vx, vy)|, m/s
  double max_angular;  // |wz|, rad/s
};

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
  bool trusted = false;
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
    if (!in.trusted) {reason = "localization not trusted"; return Decision::Hold;}
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
