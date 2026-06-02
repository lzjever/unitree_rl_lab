#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>

#include "agentic_et1_tracker/api/service.hpp"
#include "agentic_et1_tracker/reference/reference_frame_store.hpp"

namespace httplib {
struct Response;
class Server;
}

namespace agentic_et1_tracker {

inline constexpr std::size_t kHttpServerMinThreadPoolSize = 2;
inline constexpr std::size_t kHttpServerDefaultThreadPoolSize = 4;
inline constexpr std::size_t kHttpServerMaxThreadPoolSize = 4;

struct HttpServerConfig {
  std::string host{"127.0.0.1"};
  int port{8083};
  std::size_t thread_pool_size{kHttpServerDefaultThreadPoolSize};
};

HttpServerConfig normalizeHttpServerConfig(HttpServerConfig config);

class AgentHttpServer {
 public:
  AgentHttpServer(HttpServerConfig config,
                  AgentApiService& service,
                  ReferenceFrameStore* reference_store = nullptr);
  ~AgentHttpServer();

  AgentHttpServer(const AgentHttpServer&) = delete;
  AgentHttpServer& operator=(const AgentHttpServer&) = delete;

  bool start();
  void stop();
  bool isRunning() const;
  int boundPort() const;

 private:
  void installHandler();
  void writeResponse(const ApiResponse& api, httplib::Response& response) const;

  HttpServerConfig config_;
  AgentApiService& service_;
  ReferenceFrameStore* reference_store_{nullptr};
  std::unique_ptr<httplib::Server> server_;
  std::thread server_thread_;
  std::atomic<int> bound_port_{0};
};

}  // namespace agentic_et1_tracker
