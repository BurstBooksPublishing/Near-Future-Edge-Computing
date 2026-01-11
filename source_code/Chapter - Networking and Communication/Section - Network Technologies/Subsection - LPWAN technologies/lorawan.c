#include "lmic.h"
#include "hal/hal.h"

// Replace with secure element retrieval in production.
static const u1_t DEVEUI[8] = { /* big-endian */ 0x00,0x... };
static const u1_t APPEUI[8] = { 0x70,0xB3,0xD5,0x... };
static const u1_t APPKEY[16] = { 0x2B,0x7E,0x15,0x16, /* ... */ };

static osjob_t sendjob;

void os_getArtEui(u1_t* buf) { memcpy(buf, APPEUI, 8); }
void os_getDevEui(u1_t* buf) { memcpy(buf, DEVEUI, 8); }
void os_getDevKey(u1_t* buf) { memcpy(buf, APPKEY, 16); }

void do_send(osjob_t* j){
    static uint8_t payload[32] = "edge-sensor:payload"; // fill sensor data
    if (LMIC.opmode & OP_TXRXPEND) return; // radio busy
    LMIC_setTxData2(1, payload, sizeof(payload)-1, 0); // port 1, unconfirmed
    // LMIC will handle air-time; enable ADR if appropriate
}

void onEvent(ev_t ev){
    switch(ev){
    case EV_JOINED:
        LMIC_setLinkCheckMode(0); // disable link check to save duty cycle
        do_send(&sendjob);
        break;
    case EV_TXCOMPLETE:
        // schedule deep sleep or prepare next sample
        break;
    default: break;
    }
}

int main(){
    hal_init();
    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1/100); // minor skew compensation
    // Enable ADR by default for networks supporting it
    LMIC_enableADR();
    // Start OTAA join
    LMIC_startJoining();
    while(1){
        os_runloop_once(); // integrate with RTOS event loop in production
    }
    return 0;
}