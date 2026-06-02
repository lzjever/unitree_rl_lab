#include "agentic_et1_tracker/http/server.hpp"

#include <algorithm>
#include <utility>

#include <httplib.h>

#include "agentic_et1_tracker/reference/reference_frame_json.hpp"

namespace agentic_et1_tracker {
namespace {

constexpr const char* kJsonContentType = "application/json";

}  // namespace

HttpServerConfig normalizeHttpServerConfig(HttpServerConfig config) {
  config.thread_pool_size =
      std::clamp(config.thread_pool_size, kHttpServerMinThreadPoolSize,
                 kHttpServerMaxThreadPoolSize);
  return config;
}

AgentHttpServer::AgentHttpServer(HttpServerConfig config,
                                 AgentApiService& service,
                                 ReferenceFrameStore* reference_store)
    : config_(normalizeHttpServerConfig(std::move(config))),
      service_(service),
      reference_store_(reference_store),
      server_(std::make_unique<httplib::Server>()) {
  server_->new_task_queue = [threads = config_.thread_pool_size] {
    return new httplib::ThreadPool(threads);
  };
  installHandler();
}

AgentHttpServer::~AgentHttpServer() { stop(); }

bool AgentHttpServer::start() {
  if (isRunning() || server_thread_.joinable()) {
    return false;
  }

  int port = config_.port;
  bool bound = false;
  if (port == 0) {
    port = server_->bind_to_any_port(config_.host);
    bound = port > 0;
  } else {
    bound = server_->bind_to_port(config_.host, port);
  }

  if (!bound) {
    bound_port_.store(0);
    return false;
  }

  bound_port_.store(port);
  server_thread_ = std::thread([this] { server_->listen_after_bind(); });
  server_->wait_until_ready();
  return isRunning();
}

void AgentHttpServer::stop() {
  if (server_) {
    server_->stop();
  }
  if (server_thread_.joinable()) {
    server_thread_.join();
  }
  bound_port_.store(0);
}

bool AgentHttpServer::isRunning() const { return server_ && server_->is_running(); }

int AgentHttpServer::boundPort() const { return bound_port_.load(); }

void AgentHttpServer::installHandler() {
  auto handler = [this](const httplib::Request& request, httplib::Response& response) {
    const ApiResponse api = service_.handle({request.method, request.target, request.body});
    writeResponse(api, response);
  };

  server_->Get("/health", handler);
  server_->Get("/status", handler);
  server_->Get("/_sim/reference_frame",
               [this](const httplib::Request&, httplib::Response& response) {
                 if (reference_store_ == nullptr) {
                   response.status = 404;
                   response.set_content(
                       nlohmann::json{{"ok", false},
                                      {"error", {{"code", "NOT_FOUND"}}}}
                           .dump(),
                       kJsonContentType);
                   return;
                 }
                 response.status = 200;
                 response.set_content(
                     referenceFrameSnapshotJson(reference_store_->snapshot()).dump(),
                     kJsonContentType);
               });
  server_->Post("/execute", handler);
  server_->Post("/idle", handler);
  server_->Post("/stop", handler);
  server_->Post("/fixstand", handler);
  server_->Post("/standby_velocity", handler);
  server_->set_error_handler(
      [this](const httplib::Request& request, httplib::Response& response) {
        if (!response.body.empty()) {
          return;
        }
        const ApiResponse api =
            service_.handle({request.method, request.target, request.body});
        writeResponse(api, response);
      });
}

void AgentHttpServer::writeResponse(const ApiResponse& api, httplib::Response& response) const {
  response.status = api.status;
  response.set_content(api.body.dump(), kJsonContentType);
}

}  // namespace agentic_et1_tracker
