// firmware_sil/hal/network_stubs.h
//
// Placeholder for stubbing WiFi/WebSocketsServer when (later) compiling the
// firmware's command-dispatch path. NOT used in Step 1 of the SIL plan:
// network.cpp is excluded from the build, and spinal_cord/servo/leg never
// include WiFi/WebSockets. The Python `websockets` server (SIL plan §5) takes
// the transport role instead. Kept as a marker for the structure; fill in if a
// future step compiles network.cpp's dispatch.
#pragma once
