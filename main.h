#ifndef __MAIN_H__
#define __MAIN_H__

typedef struct {
    uint64_t mon_hw_monotonic;
    uint16_t agc, noise;
    uint8_t jam_cw, jam_bb;

    uint64_t mon_span_monotonic;
    uint8_t spectrum[256];
    uint32_t span; // Hz
    uint32_t res; // Hz
    uint32_t center; // Hz
    uint8_t pga; // dB

    uint64_t nav_pvt_monotonic;
    bool time_valid;
    uint64_t gnss_timestamp;
    int32_t lat, lon, alt;
    uint32_t h_acc, v_acc;

    uint8_t svs_acquired, svs_locked, svs_nav;
} jammon_datapoint_t;


#endif /* __MAIN_H__ */