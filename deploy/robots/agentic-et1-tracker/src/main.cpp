#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
#include <string>
#include <thread>
#include <utility>

#include "agentic_et1_tracker/app/app_config.hpp"
#include "agentic_et1_tracker/app/app_runner.hpp"

namespace {

std::atomic<bool> g_stop_requested{false};

void requestStop(int) { g_stop_requested.store(true); }

void printUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0 << " --config PATH\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    printUsage(argv[0]);
    return 0;
  }
  if (argc != 3 || std::string(argv[1]) != "--config") {
    printUsage(argv[0]);
    return 2;
  }

  try {
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    auto config = agentic_et1_tracker::loadAppConfig(argv[2]);
    const std::string host = config.http.host;
    agentic_et1_tracker::AppRunner runner(std::move(config));
    if (!runner.start()) {
      std::cerr << "failed to start agentic-et1-tracker HTTP server\n";
      return 1;
    }

    std::cerr << "agentic-et1-tracker listening on " << host << ":" << runner.boundPort()
              << "\n";
    while (!g_stop_requested.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    runner.stop();
    return 0;
  } catch (const agentic_et1_tracker::ConfigError& e) {
    std::cerr << e.what() << "\n";
    return 2;
  } catch (const std::exception& e) {
    std::cerr << "fatal: " << e.what() << "\n";
    return 1;
  }
}
