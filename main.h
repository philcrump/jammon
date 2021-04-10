#ifndef __MAIN_H__
#define __MAIN_H__

typedef struct {
    uint64_t mon_rf_monotonic;
    uint16_t agc, noise;
    uint8_t jam_cw, jam_bb;
    uint16_t agc2, noise2;
    uint8_t jam_cw2, jam_bb2;

    uint64_t mon_span_monotonic;
    uint8_t spectrum[256];
    uint32_t span; // Hz
    uint32_t res; // Hz
    uint32_t center; // Hz
    uint8_t pga; // dB

    bool multiband;
    uint8_t spectrum2[256]; // Only used in multi-band (eg. F9)
    uint32_t span2; // Hz
    uint32_t res2; // Hz
    uint32_t center2; // Hz
    uint8_t pga2; // dB

    uint64_t nav_pvt_monotonic;
    bool time_valid;
    uint64_t gnss_timestamp;
    int32_t lat, lon, alt;
    uint32_t h_acc, v_acc;

    uint64_t nav_sat_monotonic;
    uint8_t svs_nav;

    uint64_t nav_sig_monotonic;
    uint8_t svs_acquired_l1;
    uint8_t svs_acquired_l2; // Only used in multi-band (eg. F9)
    uint8_t svs_locked_l1;
    uint8_t svs_locked_l2; // Only used in multi-band (eg. F9)
} jammon_datapoint_t;


#endif /* __MAIN_H__ */