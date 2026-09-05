/* A TCP echo server on TI's NDK, running on the emulated C6455.
 *
 * SYS/BIOS starts, the NDK's generated stack thread brings the stack up with the static address
 * from app.cfg, and the hook below starts a task that listens on port 3000 and echoes whatever a
 * host process sends. Nothing in the network path is this project's code except the NIMU driver in
 * nimu_emac.c: the sockets, TCP, IP and ARP are all TI's.
 *
 * The hook also registers three of TI's own server daemons out of `tools/servers` - the ones the
 * `servers_min_ipv4` library this firmware already links has always contained and nothing here
 * had ever started. Their protocols and their answers are TI's, not this project's, which is what
 * makes them worth running: `dtask_tcp_datasrv` on port 1000 takes a four-byte little-endian
 * length and replies with that many bytes out of a buffer it fills with `' ' + i`, in 512-byte
 * chunks; `dtask_tcp_echo` on port 7 echoes a stream; `dtask_udp_echo` on port 7 echoes
 * datagrams. A host peer that speaks those protocols is checking TI's code against TI's code.
 */

#include <netmain.h>
#include <stdio.h>
#include <string.h>

#include <nettools/inc/daemonif.h>
#include <tools/servers.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#define ECHO_PORT 3000
#define UDP_PORT 3001
#define NOCOPY_PORT 3002
/* How many datagrams the UDP echo task serves before it stops. One is what the single-exchange
 * suite case needs; the bandwidth cases pace a burst of up to a thousand, so the pool is sized
 * for them and a scripted run still terminates on its own. */
#ifndef UDP_DATAGRAMS
#define UDP_DATAGRAMS 2048
#endif
#define ECHO_BUFFER 1024

/* How many connections to serve before the firmware stops, so a scripted run terminates. */
#ifndef ECHO_CONNECTIONS
#define ECHO_CONNECTIONS 1
#endif

static void echo_task(void);
static void nocopy_task(void);
static void udp_echo_task(void);

/* NDK hooks named in app.cfg. */
/* A page for the NDK's HTTP server, in its embedded file system. The server is configured in
 * app.cfg; all the application does is put a file where it can find it. */
static char page[] =
    "<html><body><h1>c64xemu</h1><p>NDK on the emulated C6455.</p></body></html>";

/* The daemon handles, kept so `app_network_close` can hand them back. */
static HANDLE datasrv_daemon;
static HANDLE echosrv_daemon;
static HANDLE udpecho_daemon;

void app_network_open(void)
{
    efs_createfile("index.html", (INT32)(sizeof(page) - 1u), (UINT8 *)page);
    TaskCreate(echo_task, "echo", OS_TASKPRINORM, 0x4000, 0, 0, 0);
    TaskCreate(udp_echo_task, "udp", OS_TASKPRINORM, 0x4000, 0, 0, 0);
    TaskCreate(nocopy_task, "nocopy", OS_TASKPRINORM, 0x4000, 0, 0, 0);
    /* TI's own daemons, on the ports TI's own clients connect to: `datasrv` at 1000 is what
     * `winapps/recv.c` requests a byte count from, and the two echo servers at 7 are what
     * `winapps/echoc.c` and `winapps/testudp.c` talk to. One connection at a time is all a
     * suite case needs and it keeps the frame pool clear for the paced cases. */
    datasrv_daemon =
        DaemonNew(SOCK_STREAM, 0, 1000, dtask_tcp_datasrv, OS_TASKPRINORM, 0x1000, 0, 1);
    /* The echo daemon reads with `recvnc`, so it needs the no-copy stream socket its own
     * header comment names - "dtask_tcp_echo() - TCP Echo Server Daemon Function
     * (SOCK_STREAMNC)". On an ordinary SOCK_STREAM the first `recvnc` fails and it closes. */
    echosrv_daemon =
        DaemonNew(SOCK_STREAMNC, 0, 7, dtask_tcp_echo, OS_TASKPRINORM, 0x1000, 0, 1);
    udpecho_daemon = DaemonNew(SOCK_DGRAM, 0, 7, dtask_udp_echo, OS_TASKPRINORM, 0x1000, 0, 1);
    printf("servers: ti daemons on ports 7 and 1000\n");
    fflush(stdout);
}

void app_network_close(void)
{
    if (datasrv_daemon) {
        DaemonFree(datasrv_daemon);
    }
    if (echosrv_daemon) {
        DaemonFree(echosrv_daemon);
    }
    if (udpecho_daemon) {
        DaemonFree(udpecho_daemon);
    }
}

void app_ip_address(IPN address, uint interface, uint added)
{
    char text[16];

    NtIPN2Str(address, text);
    printf("ndk: interface %u %s %s\n", interface, added ? "up with" : "lost", text);
    fflush(stdout);
}

/* A datagram echo, which is the socket API's UDP path rather than the stack's own use of it for
 * DHCP: one datagram in, the same bytes back to whoever sent them. */
static void udp_echo_task(void)
{
    SOCKET listener;
    struct sockaddr_in address;
    struct sockaddr_in peer;
    int peer_length;
    char buffer[512];
    int received;
    unsigned int served = 0;

    fdOpenSession(TaskSelf());
    listener = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (listener == INVALID_SOCKET) {
        printf("udp: socket failed\n");
        fdCloseSession(TaskSelf());
        return;
    }
    mmZeroInit(&address, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(UDP_PORT);
    if (bind(listener, (struct sockaddr *)&address, sizeof(address)) < 0) {
        printf("udp: bind failed\n");
        fdClose(listener);
        fdCloseSession(TaskSelf());
        return;
    }
    printf("udp: listening on port %d\n", UDP_PORT);
    fflush(stdout);
    while (served < UDP_DATAGRAMS) {
        peer_length = sizeof(peer);
        received = (int)recvfrom(listener, buffer, sizeof(buffer), 0,
                                 (struct sockaddr *)&peer, &peer_length);
        if (received <= 0) {
            break;
        }
        sendto(listener, buffer, received, 0, (struct sockaddr *)&peer, peer_length);
        served++;
        /* Only the first datagram prints: a paced burst would otherwise flood the host
         * console pipe faster than the emulator drains it and stall the run. */
        if (served == 1) {
            printf("udp: echoed %d bytes\n", received);
            fflush(stdout);
        }
    }
    printf("udp: served %d datagrams\n", served);
    fflush(stdout);
    fdClose(listener);
    fdCloseSession(TaskSelf());
}

/* The stack's zero-copy receive path, which nothing here had executed. `recvncfrom` hands back a
 * pointer into the stack's own receive buffer and a handle that must be returned with
 * `recvncfree`, so the datagram is echoed out of stack memory rather than out of a task buffer.
 * The path is datagram-only: SockRecvNC refuses a socket that is not an atomic reader, which is
 * why this is a UDP socket and not the TCP echo above. */
static void nocopy_task(void)
{
    SOCKET listener = INVALID_SOCKET;
    struct sockaddr_in address;
    int served = 0;

    fdOpenSession(TaskSelf());

    listener = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (listener == INVALID_SOCKET) {
        printf("nocopy: socket failed\n");
        goto done;
    }
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(NOCOPY_PORT);
    if (bind(listener, (struct sockaddr *)&address, sizeof(address)) < 0) {
        printf("nocopy: bind failed\n");
        goto done;
    }
    printf("nocopy: listening on port %d\n", NOCOPY_PORT);
    fflush(stdout);

    while (served < 1) {
        struct sockaddr_in peer;
        int peer_length = sizeof(peer);
        unsigned char *data = NULL;
        HANDLE frame = NULL;
        int received = recvncfrom(listener, (void **)&data, 0,
                                  (struct sockaddr *)&peer, &peer_length, &frame);

        if (received <= 0) {
            printf("nocopy: recvncfrom returned %d\n", received);
            fflush(stdout);
            break;
        }
        /* Echo straight out of the stack's buffer, then hand it back. */
        if (sendto(listener, (char *)data, received, 0,
                   (struct sockaddr *)&peer, peer_length) < 0) {
            printf("nocopy: sendto failed\n");
        } else {
            printf("nocopy: echoed %d bytes without a copy\n", received);
        }
        fflush(stdout);
        recvncfree(frame);
        served++;
    }

done:
    if (listener != INVALID_SOCKET) {
        fdClose(listener);
    }
    fdCloseSession(TaskSelf());
}

static void echo_task(void)
{
    SOCKET listener = INVALID_SOCKET;
    struct sockaddr_in address;
    int served = 0;

    /* Every task that uses sockets opens a file descriptor session first. */
    fdOpenSession(TaskSelf());

    listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listener == INVALID_SOCKET) {
        printf("echo: socket failed\n");
        goto done;
    }
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(ECHO_PORT);
    if (bind(listener, (struct sockaddr *)&address, sizeof(address)) < 0) {
        printf("echo: bind failed\n");
        goto done;
    }
    if (listen(listener, 2) < 0) {
        printf("echo: listen failed\n");
        goto done;
    }
    printf("echo: listening on port %d\n", ECHO_PORT);
    fflush(stdout);

    while (served < ECHO_CONNECTIONS) {
        struct sockaddr_in peer;
        int peer_length = sizeof(peer);
        SOCKET session = accept(listener, (struct sockaddr *)&peer, &peer_length);
        char buffer[ECHO_BUFFER];
        int total = 0;

        if (session == INVALID_SOCKET) {
            printf("echo: accept failed\n");
            break;
        }
        {
            char text[16];

            NtIPN2Str(peer.sin_addr.s_addr, text);
            printf("echo: connection from %s:%d\n", text, ntohs(peer.sin_port));
        }
        fflush(stdout);
        for (;;) {
            int received = recv(session, buffer, sizeof(buffer), 0);

            if (received <= 0) {
                break;
            }
            total += received;
            if (send(session, buffer, received, 0) < 0) {
                break;
            }
        }
        printf("echo: connection closed after %d bytes\n", total);
        fflush(stdout);
        fdClose(session);
        served++;
    }

done:
    if (listener != INVALID_SOCKET) {
        fdClose(listener);
    }
    fdCloseSession(TaskSelf());
    printf("echo: done after %d connection(s)\n", served);
    fflush(stdout);
    /* Let the run finish: the emulator stops when main's exit path is reached. */
    BIOS_exit(0);
}

/* The console library prints this at the top of a session and of its help listing; the
 * application owns it, which is why the NDK declares it extern. */
char *VerStr = "\nc64xemu NDK console\n";

int main(void)
{
    printf("c64xemu ndk firmware starting\n");
    fflush(stdout);
    BIOS_start();
    return 0;
}
