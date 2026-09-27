#ifndef AOS_EVENTS_WINSOCK_INIT_H_
#define AOS_EVENTS_WINSOCK_INIT_H_

// Windows-only: every translation unit that creates a SOCKET has to make sure
// Winsock has been started first, and there is no natural single owner of that
// -- Aio and Pipe each create sockets, and either can be the first or the last
// one alive.

#if defined(_WIN32)

namespace aos {

// Starts Winsock exactly once per process, on first call.
//
// Deliberately never calls WSACleanup().  WSAStartup/WSACleanup are
// refcounted, so pairing them per-object means the count reaches zero
// whenever the last such object is destroyed -- which deinitializes Winsock
// while unrelated sockets may still be about to be created, and the next
// socket() fails with WSANOTINITIALISED (10093).  Leaving Winsock up for the
// life of the process avoids that ordering hazard entirely, and costs
// nothing: the OS reclaims everything at exit.
void EnsureWinsockInitialized();

}  // namespace aos

#endif  // _WIN32

#endif  // AOS_EVENTS_WINSOCK_INIT_H_
