/* safety_monitor.c — aggregates heartbeats from perception and sensor tasks */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdatomic.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>

static atomic_uint_fast64_t last_hb_ns = 0;    /* monotonic timestamp of last heartbeat */
static atomic_bool shutdown_req = false;

/* get current monotonic time in nanoseconds */
static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

/* public API for other processes/threads to post a heartbeat */
void post_heartbeat(void) {
    atomic_store_explicit(&last_hb_ns, now_ns(), memory_order_release);
}

/* background thread: monitors heartbeat and triggers failover if timeout exceeded */
static void* monitor_thread(void* arg) {
    (void)arg;
    const uint64_t timeout_ns = 50U * 1000000U; /* 50 ms detection window */
    while (!atomic_load_explicit(&shutdown_req, memory_order_acquire)) {
        uint64_t t0 = atomic_load_explicit(&last_hb_ns, memory_order_acquire);
        uint64_t t1 = now_ns();
        if (t0 == 0 || (t1 - t0) > timeout_ns) {
            /* failover action: log, demote perception outputs, command safe behavior */
            fprintf(stderr, "SAFETY: heartbeat timeout; entering safe state\n");
            /* Example: send command to brake interface or reduce automation level */
            /* production: use secure IPC or CAN/FD driver to issue safe-state command */
            /* backoff to avoid tight loop */
            sleep(1);
        } else {
            /* sleep small interval to reduce CPU usage; high-res sleep recommended */
            struct timespec req = {.tv_sec = 0, .tv_nsec = 5U * 1000000U}; /* 5 ms */
            nanosleep(&req, NULL);
        }
    }
    return NULL;
}

/* install signal handlers for graceful shutdown in production */
static void sigint_handler(int signum) { (void)signum; atomic_store(&shutdown_req, true); }

int main(void) {
    signal(SIGINT, sigint_handler);
    pthread_t mon;
    if (pthread_create(&mon, NULL, monitor_thread, NULL) != 0) return EXIT_FAILURE;
    /* Example: simulate heartbeats from perception task */
    for (int i = 0; i < 100 && !atomic_load(&shutdown_req); ++i) {
        post_heartbeat();
        usleep(20000); /* 20 ms */
    }
    atomic_store(&shutdown_req, true);
    pthread_join(mon, NULL);
    return EXIT_SUCCESS;
}