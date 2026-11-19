#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace uav_ugv_msgs
{
inline int64_t trace_steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline const std::string & trace_host_id()
{
  static const std::string value = []() {
      std::ifstream file("/proc/sys/kernel/random/boot_id");
      std::string id;
      std::getline(file, id);
      return id.empty() ? std::string("unverified") : id;
    }();
  return value;
}

// Encoding only. No ROS, simulator, filesystem writes, or numerical policy.
class EvaluationJson
{
public:
  EvaluationJson() {out_ << '{';}

  static std::string quote(const std::string & text)
  {
    std::ostringstream out;
    out << '"';
    for (const unsigned char ch : text) {
      if (ch == '"' || ch == '\\') {out << '\\' << ch;} else if (ch < 0x20) {
        out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(ch);
      } else {out << ch;}
    }
    out << '"';
    return out.str();
  }

  EvaluationJson & text(const std::string & key, const std::string & value)
  {field(key); out_ << quote(value); return *this;}

  EvaluationJson & integer(const std::string & key, int64_t value)
  {field(key); out_ << value; return *this;}

  EvaluationJson & number(const std::string & key, double value)
  {
    field(key);
    if (std::isfinite(value)) {out_ << std::setprecision(17) << value;} else {out_ << "null";}
    return *this;
  }

  EvaluationJson & boolean(const std::string & key, bool value)
  {field(key); out_ << (value ? "true" : "false"); return *this;}

  std::string str() const {return out_.str() + '}';}

private:
  void field(const std::string & key)
  {
    if (!first_) {out_ << ',';}
    first_ = false;
    out_ << quote(key) << ':';
  }
  std::ostringstream out_;
  bool first_{true};
};
}  // namespace uav_ugv_msgs
