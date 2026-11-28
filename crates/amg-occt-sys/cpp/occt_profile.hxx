// Phase timing of the long kernel ops (S20 KH-20), for profiling only.
//
// With the environment variable AMG_OCCT_PROFILE=1, the ops that use it print one line per
// phase to stderr: their own steps (`Phases::mark`) and every change of OCCT's progress scope
// chain inside the algorithm (`profile_scope`, called from the progress indicators' Show). The
// time printed on a line is the time since the previous line, so it belongs to the phase named
// on the previous line. Without the variable nothing is printed and the cost is one cached
// flag check per call.
#pragma once

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>

#include <Message_ProgressScope.hxx>

namespace amg::occt::profile {

inline bool enabled() {
  static const bool on = [] {
    const char *v = std::getenv("AMG_OCCT_PROFILE");
    return v != nullptr && v[0] == '1';
  }();
  return on;
}

struct State {
  std::mutex mutex;
  std::chrono::steady_clock::time_point last = std::chrono::steady_clock::now();
  std::string chain;
};

inline State &state() {
  static State s;
  return s;
}

// Prints `what` with the time since the previous line.
inline void line(const std::string &what) {
  State &s = state();
  const auto now = std::chrono::steady_clock::now();
  const double ms = std::chrono::duration<double, std::milli>(now - s.last).count();
  s.last = now;
  std::fprintf(stderr, "[amg-prof] %9.3f ms | %s\n", ms, what.c_str());
}

// The op's own steps.
class Phases {
public:
  explicit Phases(const char *op) : op_(op) {
    if (enabled()) {
      State &s = state();
      std::lock_guard<std::mutex> lock(s.mutex);
      s.chain.clear();
      line(std::string(op_) + ": start");
    }
  }

  void mark(const char *step) const {
    if (enabled()) {
      State &s = state();
      std::lock_guard<std::mutex> lock(s.mutex);
      s.chain.clear();
      line(std::string(op_) + ": " + step);
    }
  }

private:
  const char *op_;
};

// A change of the progress scope chain (names from the outermost scope in).
inline void scope(const Message_ProgressScope &scope) {
  if (!enabled()) {
    return;
  }
  std::string chain;
  for (const Message_ProgressScope *p = &scope; p != nullptr; p = p->Parent()) {
    const char *name = p->Name();
    if (name != nullptr && name[0] != '\0') {
      chain = chain.empty() ? std::string(name) : std::string(name) + " / " + chain;
    }
  }
  State &s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  if (chain != s.chain) {
    s.chain = chain;
    line("  occt: " + chain);
  }
}

} // namespace amg::occt::profile
