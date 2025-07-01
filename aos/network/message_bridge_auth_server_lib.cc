#include "aos/network/message_bridge_auth_server_lib.h"

#include <cstdio>
#include <fstream>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "grpc/grpc.h"
#include "grpcpp/server_context.h"

namespace aos::message_bridge::auth {

namespace {

using ::grpc::ServerContext;
using ::grpc::Status;

constexpr int kSctpKeySize = 16;

std::string GenerateSecureRandomSequence(size_t count) {
  std::ifstream rng("/dev/random", std::ios::in | std::ios::binary);
  ABSL_CHECK(rng) << "Unable to open /dev/random";
  std::string out(count, 0);
  rng.read(out.data(), count);
  ABSL_CHECK(rng) << "Couldn't read from random device";
  rng.close();
  return out;
}
}  // namespace

// This class implements the SctpConfigService. It securely generates an SCTP
// authentication key and sends it to the clients that request it.
SctpConfigServer::SctpConfigServer()
    : active_key_(GenerateSecureRandomSequence(kSctpKeySize)) {}

Status SctpConfigServer::GetActiveKey(ServerContext * /*context*/,
                                      const SctpKeyRequest * /*request*/,
                                      SctpKeyResponse *response) {
  ABSL_VLOG(1) << "Got request for SCTP authentication key";
  response->set_key(active_key_);
  return Status::OK;
}

}  // namespace aos::message_bridge::auth
