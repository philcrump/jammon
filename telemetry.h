#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

void udp_send_msgpack(char *host, uint16_t port, jammon_datapoint_t *jammon_datapoint_ptr);

#endif /* __TELEMETRY_H__ */