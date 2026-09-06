# Simulator HOLD routing

## Simulator routing

Each simulated robot exclusively binds a loopback UDP port computed from its
UTF-8 robot ID: `20000 + FNV1a32(id) % 20000`. FNV-1a uses offset basis
2166136261 and prime 16777619, with 32-bit unsigned overflow. Core uses the same
mapping for simulation resources; physical firmware retains port 19520.
IDs must be nonempty and shorter than 32 bytes. The datagram still contains the
exact robot ID, and an endpoint rejects frames addressed to another ID.

This permits multiple processes without `SO_REUSEADDR` packet competition.
An occupied port (including a hash collision or duplicate robot ID) fails
registration explicitly; choose a distinct ID instead of sharing an endpoint.
Sender and simulator updates must be delivered together. This local simulator
control protocol is not an authenticated remote control interface.
