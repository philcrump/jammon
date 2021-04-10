#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <errno.h>

#include "main.h"
#include "telemetry.h"

static bool app_exit = false;
static bool verbose = false;
static bool multiband = false;

#define BUFFER_LENGTH   2048
static uint8_t buffer[BUFFER_LENGTH];

/* Reset */
static const uint8_t ubx_cfg_rst[] = {
    0xb5, 0x62,
    0x06, 0x04,
    0x04, 0x00,
    0x00, 0x00, /* BBR Sections to clear (0x0000 == Hot Start) */
    0x04, /* 0x04 = Hardware reset (watchdog) aftershutdown */
    0x00, /* [Reserved] */
    0x12, 0x6c /* Checksum */
};

/* Disable NMEA */
static const uint8_t ubx_disable_nmea_usb[] = {
    0xb5, 0x62,
    0x06, 0x00,
    0x14, 0x00,
    0x03 /* USB */ ,0x00,
    0x00,0x00,
    0x00,0x00,
    0x00,0x00,
    0x00,0x00,0x00,0x00,
    0x03,0x00,0x01,0x00,
    0x00,0x00,0x00,0x00,
    0x21, 0xa2 /* Checksum */
};

/* Enable Interference Detection  */
static const uint8_t ubx_cfg_valset_enable_itfm[] = {
    0xb5, 0x62,
    0x06, 0x8a, /* UBX-CFG-VALSET */
    0x0e, 0x00,
    0x00, 0x01, 0x00, 0x00,
    0x0d, 0x00, 0x41, 0x10, 0x01, // Enable ITFM ( 0x1041000d )
    0x10, 0x00, 0x41, 0x20, 0x02, // Set Active Antenna ( 0x20410010 )
    0x71, 0xd8 /* Checksum */
};

/* Set Automotive dynamic model */
static const uint8_t ubx_set_nav_automotive[] = {
    0xb5, 0x62,
    0x06, 0x24,
    0x24, 0x00,
    0x01, 0x00, /* Bitmask of settings to apply: bit 0 = dynamic model only */
    0x04, /* Dynamic Model = 4 (Automotive) */
    0x00, /* Fix Type */
    0x00, 0x00, 0x00, 0x00, /* 2D Altitude Value */
    0x00, 0x00, 0x00, 0x00, /* 2D Altitude Variance */
    0x00, /* Minimum GNSS Satellite Elevation */
    0x00, /* Reserved */
    0x00, 0x00, /* Position DOP Mask */
    0x00, 0x00, /* Time DOP Mask */
    0x00, 0x00, /* Position Accuracy Mask */
    0x00, 0x00, /* Time Accuracy Mask */
    0x00, /* Static hold threshold */
    0x00, /* DGNSS Timeout */
    0x00, /* Min Satellites for Fix */
    0x00, /* Min C/N0 Threshold for Satellites */
    0x00, 0x00, /* Reserved */
    0x00, 0x00, /* Static Hold Distance Threshold */
    0x00, /* UTC Standard (Automatic) */
    0x00, 0x00, 0x00, 0x00, 0x00, /* Reserved */
    0x53, 0x70 /* Checksum */
};

/* Time & Position Message */
static const uint8_t ubx_enable_nav_pvt[] = {
    0xb5, 0x62,
    0x06, 0x01, /* UBX-CFG-MSG */
    0x08, 0x00,
    0x01, 0x07, /* Enable UBX-NAV-PVT */
    0x00, /* Port 0 (I2C) */
    0x00, /* Port 1 (UART/UART1) */
    0x00, /* Port 2 (UART2) */
    0x01, /* Port 3 (USB) at 1 second interval */
    0x00, /* Port 4 */
    0x00, /* Port 5 */
    0x18, 0xdf /* Checksum */
};

/* Jamming indicators Message */
static const uint8_t ubx_enable_mon_rf[] = {
    0xb5, 0x62,
    0x06, 0x01, /* UBX-CFG-MSG */
    0x08, 0x00,
    0x0A, 0x38, /* Enable UBX-MON-RF */
    0x00, /* Port 0 (I2C) */
    0x00, /* Port 1 (UART/UART1) */
    0x00, /* Port 2 (UART2) */
    0x01, /* Port 3 (USB) at 1 second interval */
    0x00, /* Port 4 */
    0x00, /* Port 5 */
    0x52, 0x7E /* Checksum */
};

typedef struct
{
    uint8_t version;
    uint8_t num_rfblocks;
    uint8_t _reserved1;
    uint8_t _reserved2;
} __attribute__((packed)) mon_rf_header_t;

typedef struct
{
    uint8_t rfblock_id;
    uint8_t flags;
    uint8_t antStatus;
    uint8_t antPower;
    uint32_t postStatus;
    uint8_t _reserved1;
    uint8_t _reserved2;
    uint8_t _reserved3;
    uint8_t _reserved4;
    uint16_t noisePerMS;
    uint16_t agcCnt;
    uint8_t jamInd; /* CW jamming measure */
    int8_t ofsI;
    uint8_t magI;
    int8_t ofsQ;
    uint8_t magQ;
    uint8_t _reserved5;
    uint8_t _reserved6;
    uint8_t _reserved7;
} __attribute__((packed)) mon_rf_rfblock_t;


/* Crude Spectrum Analyzer */
static const uint8_t ubx_enable_mon_span[] = {
    0xb5, 0x62,
    0x06, 0x01, /* UBX-CFG-MSG */
    0x08, 0x00,
    0x0A, 0x31, /* Enable UBX-MON-SPAN */
    0x00, /* Port 0 (I2C) */
    0x00, /* Port 1 (UART/UART1) */
    0x00, /* Port 2 (UART2) */
    0x01, /* Port 3 (USB) at 1 second interval */
    0x00, /* Port 4 */
    0x00, /* Port 5 */
    0x4b, 0x4d /* Checksum */
};

typedef struct
{
    uint8_t version;
    uint8_t num_rfblocks;
    uint8_t _reserved1;
    uint8_t _reserved2;
} __attribute__((packed)) mon_span_header_t;

typedef struct
{
    uint8_t spectrum[256];
    uint32_t span;
    uint32_t res;
    uint32_t center;
    uint8_t pga;
    uint8_t _reserved1;
    uint8_t _reserved2;
    uint8_t _reserved3;
} __attribute__((packed)) mon_span_rfblock_t;

typedef struct
{
    uint32_t itow;
    uint16_t year;
    uint8_t month; // jan = 1
    uint8_t day;
    uint8_t hour; // 24
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixtype;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numsv;
    //      1e-7       mm       mm
    int32_t lon, lat, height, hMSL;
    //        mm     mm
    uint32_t hAcc, vAcc;
    //      mm/s   mm/s  mm/s  mm/s
    int32_t velN, velE, velD, gSpeed; // millimeters
} __attribute__((packed)) nav_pvt_t;

/* SV Signals */
static const uint8_t ubx_enable_nav_sat[] = {
    0xb5, 0x62,
    0x06, 0x01, /* UBX-CFG-MSG */
    0x08, 0x00,
    0x01, 0x35, /* Enable UBX-NAV-SAT */
    0x00, /* Port 0 (I2C) */
    0x00, /* Port 1 (UART/UART1) */
    0x00, /* Port 2 (UART2) */
    0x01, /* Port 3 (USB) at 1 second interval */
    0x00, /* Port 4 */
    0x00, /* Port 5 */
    0x46, 0x21 /* Checksum */
};

typedef struct
{
    uint32_t itow;
    uint8_t version;
    uint8_t num_svs;
    uint8_t _reserved1;
    uint8_t _reserved2;
} __attribute__((packed)) nav_sat_header_t;

typedef struct
{
    uint8_t gnss_id;
    uint8_t sv_id;
    uint8_t cn0;
    int8_t elevation;
    int16_t azimuth;
    int16_t psr_res;
    uint32_t flags;
} __attribute__((packed)) nav_sat_sv_t;

/* SV Signals */
static const uint8_t ubx_enable_nav_sig[] = {
    0xb5, 0x62,
    0x06, 0x01, /* UBX-CFG-MSG */
    0x08, 0x00,
    0x01, 0x43, /* Enable UBX-NAV-SIG */
    0x00, /* Port 0 (I2C) */
    0x00, /* Port 1 (UART/UART1) */
    0x00, /* Port 2 (UART2) */
    0x01, /* Port 3 (USB) at 1 second interval */
    0x00, /* Port 4 */
    0x00, /* Port 5 */
    0x54, 0x83 /* Checksum */
};

typedef struct
{
    uint32_t itow;
    uint8_t version;
    uint8_t num_svs;
    uint8_t _reserved1;
    uint8_t _reserved2;
} __attribute__((packed)) nav_sig_header_t;

typedef struct
{
    uint8_t gnss_id;
    uint8_t sv_id;
    uint8_t sig_id;
    uint8_t freq_id;
    int16_t pr_res;
    uint8_t cn0;
    uint8_t qualInd;
    uint8_t corrSource;
    uint8_t ionoModel;
    uint16_t flags;
    uint8_t _reserved0;
} __attribute__((packed)) nav_sig_sv_t;

static uint64_t monotonic_ms(void)
{
    struct timespec tp;

    if(clock_gettime(CLOCK_MONOTONIC, &tp) != 0)
    {
        return 0;
    }

    return (uint64_t) tp.tv_sec * 1000 + tp.tv_nsec / 1000000;
}

static void sleep_ms(uint32_t _duration)
{
    struct timespec req, rem;
    req.tv_sec = _duration / 1000;
    req.tv_nsec = (_duration - (req.tv_sec*1000))*1000*1000;

    while(nanosleep(&req, &rem) == EINTR)
    {
        /* Interrupted by signal, shallow copy remaining time into request, and resume */
        req = rem;
    }
}

static bool ubx_verify_checksum(const uint8_t *buffer, int32_t buffer_size)
{
    uint32_t ck_a = 0, ck_b = 0;

    for(int i = 2; i<(buffer_size-2); i++)
    {
        ck_a += buffer[i];
        ck_b += ck_a;
    }

    return ((ck_a & 0xFF) == buffer[buffer_size-2]) && ((ck_b & 0xFF) == buffer[buffer_size-1]);
}

static const uint8_t msg_ack_header[4] = { 0xb5, 0x62, 0x05, 0x01 };
static const uint8_t msg_nack_header[4] = { 0xb5, 0x62, 0x05, 0x00 };

static uint8_t send_ubx_wait_ack(int fd, const uint8_t *buffer, uint32_t buffer_size)
{
    uint32_t k;
    int32_t device_response;
    uint8_t response_byte;
    uint8_t msg_class_id = buffer[2];
    uint8_t msg_msg_id = buffer[3];

    tcflush(fd, TCIFLUSH);
    if(write(fd, buffer, buffer_size) != (int)buffer_size)
    {
        return 3;
    }

    uint32_t response_index = 0;
    for(k = 0; (k < 2000) && !app_exit; k++)
    {
        device_response = read(fd, &response_byte, 1);
        if(device_response == 0)
        {
            fprintf(stderr, " - GNSS Device EOF (device disconnected).\n");
            return 1;
        }
        else if(device_response < 0)
        {
            fprintf(stderr, " - GNSS Device Read Error: %s\n", strerror(errno));
            return 1;
        }
        else if(device_response > 0)
        {
            if(response_index <= 3 && response_byte == msg_ack_header[response_index])
            {
                /* ACK header */
                response_index++;
            }
            else if(response_index > 3 && response_index <= 5)
            {
                response_index++;
            }
            else if(response_index == 6 && response_byte == msg_class_id)
            {
                /* Full ACK of correct class and message id */
                response_index++;
            }
            else if(response_index == 7 && response_byte == msg_msg_id)
            {
                /* Full ACK of correct class and message id */
                return 0;
            }
            else if(response_index == 3 && response_byte == msg_nack_header[response_index])
            {
                /* Got NACK */
                fprintf(stderr, " - Got NACK\n");
                return 2;
            }
            else
            {
                response_index = 0;
            }
        }
    }

    fprintf(stderr, " - Timed out\n");

    return 1;
}

static uint8_t send_ubx(int fd, const uint8_t *buffer, uint32_t buffer_size)
{
    if(write(fd, buffer, buffer_size) != (int)buffer_size)
    {
        return 3;
    }

    tcflush(fd, TCIFLUSH);
    return 0;
}

static const uint8_t msg_header[2] = { 0xb5, 0x62 };
static uint32_t wait_ubx(int fd, uint8_t *buffer)
{
    int32_t device_response;
    uint8_t response_byte;
    uint32_t response_index = 0;
    uint32_t response_length = 0;

    while(!app_exit)
    {
        device_response = read(fd, &response_byte, 1);
        if(device_response == 0)
        {
            fprintf(stderr, "GNSS Device EOF (device disconnected).\n");
            app_exit = true;
            return 0;
        }
        else if(device_response < 0)
        {
            fprintf(stderr, "GNSS Device Read Error: %s\n", strerror(errno));
            app_exit = true;
            return 0;
        }
        else if(device_response > 0)
        {
            if(response_index < 2 && response_byte == msg_header[response_index])
            {
                /* Header */
                buffer[response_index] = response_byte;
                response_index++;
            }
            else if(response_index >= 2 && response_index < 5)
            {
                /* Message Class, ID, first byte of length */
                buffer[response_index] = response_byte;
                response_index++;
            }
            else if(response_index == 5)
            {
                /* Final Length byte */
                buffer[response_index] = response_byte;
                response_length = buffer[5] << 8 | buffer[4];
                if(response_length >= BUFFER_LENGTH)
                {
                    fprintf(stderr, "Error: UBX Response too long for buffer (%d/%d)\n", response_length, BUFFER_LENGTH);
                    return 0;
                }
                response_index++;
            }
            else if(response_index > 5 && response_index < (6+2+response_length-1))
            {
                /* Data payload and first byte of checksum */
                buffer[response_index] = response_byte;
                response_index++;
            }
            else if(response_index == (6+2+response_length-1))
            {
                /* Last byte of checksum */
                buffer[response_index] = response_byte;

                if(ubx_verify_checksum(buffer, (6+2+response_length)))
                {
                    #if 0
                    printf(" - CRC OK\n");

                    for(unsigned int i = 0; i < (6+2+response_length); i++)
                    {
                        printf("%02x", buffer[i]);
                    }
                    printf("\n");
                    #endif

                    return (6+2+response_length);
                }

                /* else */
                #if 1
                printf(" - CRC fail (message: %02x, %02x)\n", buffer[2], buffer[3]);

                for(unsigned int i = 0; i < (6+2+response_length); i++)
                {
                    printf("%02x", buffer[i]);
                }
                printf("\n");
                #endif

                response_index = 0;
            }
            else
            {
                response_index = 0;
            }
        }
    }

    return 0;
}

static void process_datapoint(jammon_datapoint_t jammon_datapoint, char *udp_host, uint16_t udp_port)
{
    FILE *csv_fptr;
    char csv_filename[32];

    if(verbose)
    {
        printf("Datapoint:\n");
        printf(" - Timestamp: %"PRIu64" - %.24s\n", jammon_datapoint.gnss_timestamp, ctime((time_t *)&jammon_datapoint.gnss_timestamp));
        printf(" - Position: %d, %d, %d\n", jammon_datapoint.lat, jammon_datapoint.lon, jammon_datapoint.alt);
        printf(" - Accuracy: H: %.1fm, V: %.1fm\n", jammon_datapoint.h_acc / 1.0e3, jammon_datapoint.v_acc / 1.0e3);
        printf(" - SVs: Acquired: (%d|%d), Locked: (%d|%d), Used in Nav: %d\n", jammon_datapoint.svs_acquired_l1, jammon_datapoint.svs_acquired_l2, jammon_datapoint.svs_locked_l1, jammon_datapoint.svs_locked_l2, jammon_datapoint.svs_nav);
        printf(" - AGC: %d|%d, Noise: %d|%d\n", jammon_datapoint.agc, jammon_datapoint.agc2, jammon_datapoint.noise, jammon_datapoint.noise2);
        printf(" - L1 Jamming: CW: %d / 255, Broadband: %d / 3 (0 = invalid)\n", jammon_datapoint.jam_cw, jammon_datapoint.jam_bb);
        printf(" - L2 Jamming: CW: %d / 255, Broadband: %d / 3 (0 = invalid)\n", jammon_datapoint.jam_cw2, jammon_datapoint.jam_bb2);
    }

    if(jammon_datapoint.time_valid)
    {
        int r;
        char *csv_output_line;

        r = asprintf(&csv_output_line, "%"PRIu64",%.24s,%.5f,%.5f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
            jammon_datapoint.gnss_timestamp, ctime((time_t *)&jammon_datapoint.gnss_timestamp),
            (jammon_datapoint.lat / 1.0e7), (jammon_datapoint.lon / 1.0e7), (jammon_datapoint.alt / 1.0e3),
            jammon_datapoint.h_acc / 1.0e3, jammon_datapoint.v_acc / 1.0e3,
            jammon_datapoint.svs_acquired_l1, jammon_datapoint.svs_acquired_l2, jammon_datapoint.svs_locked_l1, jammon_datapoint.svs_locked_l2, jammon_datapoint.svs_nav,
            jammon_datapoint.agc, jammon_datapoint.noise, jammon_datapoint.jam_cw, jammon_datapoint.jam_bb,
            jammon_datapoint.agc2, jammon_datapoint.noise2, jammon_datapoint.jam_cw2, jammon_datapoint.jam_bb2
        );

        if(r < 0)
        {
            fprintf(stderr, "Error: asprintf of Log CSV line failed.\n");
        }
        else
        {
            strftime(csv_filename, 31, "log-jammon-%Y-%m-%d.csv", localtime((time_t *)&(jammon_datapoint.gnss_timestamp)));

            csv_fptr = fopen(csv_filename, "a+"); 
            if(csv_fptr != NULL)
            {
                fputs(csv_output_line, csv_fptr);
                fclose(csv_fptr);
            }
            else
            {
                fprintf(stderr, "Error: Unable to open log CSV file\n");
            }

            free(csv_output_line);
        }

        

        /* Allocate 2*256, + 1 for sprintf null termination */
        char *csv_output_spectrum = malloc((2*256)+1);

        if(csv_output_spectrum == NULL)
        {
            fprintf(stderr, "Error: Unable to allocate memory for spectrum CSV line\n");
            return;
        }

        for(int i = 0; i < 255; i++)
        {
            sprintf(&csv_output_spectrum[i*2], "%02x", jammon_datapoint.spectrum[i]);
        }

        r = asprintf(&csv_output_line, "%"PRIu64",%d,%d,%d,%d,\"%s\"\n",
            jammon_datapoint.gnss_timestamp,
            jammon_datapoint.span, jammon_datapoint.res, jammon_datapoint.center, jammon_datapoint.pga,
            csv_output_spectrum
        );
        free(csv_output_spectrum);

        if(r < 0)
        {
            fprintf(stderr, "Error: asprintf of Spectrum L1 CSV line failed.\n");
            return;
        }
        strftime(csv_filename, 33, "spectruml1-jammon-%Y-%m-%d.csv", localtime((time_t *)&(jammon_datapoint.gnss_timestamp)));

        csv_fptr = fopen(csv_filename, "a+"); 
        if(csv_fptr != NULL)
        {
            fputs(csv_output_line, csv_fptr);
            fclose(csv_fptr);
        }
        else
        {
            fprintf(stderr, "Error: Unable to open Spectrum L1 CSV file\n");
        }
        free(csv_output_line);

        if(multiband)
        {
            /* Append second spectrum to CSV line */
            csv_output_spectrum = malloc((2*256)+1);

            if(csv_output_spectrum == NULL)
            {
                fprintf(stderr, "Error: Unable to allocate memory for spectrum2 CSV line\n");
                return;
            }

            for(int i = 0; i < 255; i++)
            {
                sprintf(&csv_output_spectrum[i*2], "%02x", jammon_datapoint.spectrum2[i]);
            }

            r = asprintf(&csv_output_line, "%"PRIu64",%d,%d,%d,%d,\"%s\"\n",
                jammon_datapoint.gnss_timestamp,
                jammon_datapoint.span2, jammon_datapoint.res2, jammon_datapoint.center2, jammon_datapoint.pga2,
                csv_output_spectrum
            );
            free(csv_output_spectrum);

            if(r < 0)
            {
                fprintf(stderr, "Error: asprintf of Spectrum L2 CSV line failed.\n");
                return;
            }
            strftime(csv_filename, 33, "spectruml2-jammon-%Y-%m-%d.csv", localtime((time_t *)&(jammon_datapoint.gnss_timestamp)));

            csv_fptr = fopen(csv_filename, "a+"); 
            if(csv_fptr != NULL)
            {
                fputs(csv_output_line, csv_fptr);
                fclose(csv_fptr);
            }
            else
            {
                fprintf(stderr, "Error: Unable to open Spectrum L2 CSV file\n");
            }
            free(csv_output_line);
        }
    }

    /* Lastly, send UDP Telemetry */
    udp_send_msgpack(udp_host, udp_port, &jammon_datapoint);
}

static void open_serialDevice(int *fd_ptr, char *devName)
{
    struct termios tty;

    *fd_ptr = open(devName, O_RDWR);
    if(*fd_ptr < 0)
    {
        fprintf(stderr, "Error: Cannot open serial device '%s'\n", devName);
        return;
    }

    if (tcgetattr (*fd_ptr, &tty) != 0)
    {
        fprintf(stderr, "Error: tcgetattr\n");
        close(*fd_ptr);
        *fd_ptr = -1;
        return;
    }

    cfsetospeed (&tty, B115200);
    cfsetispeed (&tty, B115200);

    tty.c_iflag &= ~(IGNBRK | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tty.c_oflag &= ~(ONLCR | OCRNL);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cc[VMIN]  = 1; // read doesn't block
    tty.c_cc[VTIME] = 1; // 0.1 seconds read timeout

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (*fd_ptr, TCSANOW, &tty) != 0)
    {
        fprintf(stderr, "Error: tcsetattr\n");
        close(*fd_ptr);
        *fd_ptr = -1;
        return;
    }
}

void sigint_handler(int sig)
{
    (void)sig;
    app_exit = true;
}

static void usage( void )
{
    printf("Usage: jammon [-v] [-M] [-r] -d <device> -H <host> -P <port>\n");  
}
 
int main(int argc, char *argv[])
{
    int fd;
    int option = 0;

    char *devName = NULL;
    char *udp_host = NULL;
    uint16_t udp_port = 44333;
    bool rx_reset = false;

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);
   
    while((option = getopt( argc, argv, "vd:MH:P:r")) != -1)
    {
        switch(option)
        {
            case 'd':
                devName = strdup(optarg);
                printf(" * Using serial device: %s\n", devName);
                break;
            case 'v':
                verbose = true;
                printf(" * Verbose Mode Enabled\n");
                break;
            case 'M':
                multiband = true;
                printf(" * Multiband (F9) Enabled\n");
                break;
            case 'H':
                udp_host = strdup(optarg);
                printf(" * Using target host: %s\n", udp_host);
                break;
            case 'P':
                udp_port = atoi(optarg);
                printf(" * Using target port: %d\n", udp_port);
                break;
            case 'r':
                rx_reset = true;
                printf(" * Device Reset enabled\n");
                break;
            default:
                usage();
                return 0;
        }        
    }
 
    if(devName == NULL)
    {
        usage();
        return 1;
    }

    /* Open Serial Port */
    open_serialDevice(&fd, devName);       
    if(fd < 0)
    {
        fprintf(stderr, "Error: Cannot open serial device '%s'\n", devName);
        free(devName);
        return 1;
    }

    if(rx_reset)
    {
        printf("Resetting GNSS Receiver..\n");

        if(0 != send_ubx(fd, ubx_cfg_rst, sizeof(ubx_cfg_rst)))
        {
            fprintf(stderr, "Failed to send reset command\n");
            close(fd);
            return 1;
        }
        close(fd);

        printf("Waiting 5 seconds for receiver to reboot..\n");
        sleep_ms(5000);

        printf("Re-opening serial port after reset..\n");
        open_serialDevice(&fd, devName);    
        if(fd < 0)
        {
            fprintf(stderr, "Error: Cannot re-open serial device '%s'\n", devName);
            free(devName);
            return 1;
        }
    }

    free(devName);

    printf("Configuring..\n");

    if(verbose) printf(" - Disabling NMEA on usb..\n");
    if(0 != send_ubx_wait_ack(fd, ubx_disable_nmea_usb, sizeof(ubx_disable_nmea_usb)))
    {
        fprintf(stderr, "Failed to disable NMEA on USB\n");
        close(fd);
        return 1;
    }

    if(verbose) printf(" - Enabling interference detection..\n");
    if(0 != send_ubx_wait_ack(fd, ubx_cfg_valset_enable_itfm, sizeof(ubx_cfg_valset_enable_itfm)))
    {
        fprintf(stderr, "Failed to enable interference detection\n");
        close(fd);
        return 1;
    }

    if(verbose) printf(" - Setting automotive mode..\n");
    if(0 != send_ubx_wait_ack(fd, ubx_set_nav_automotive, sizeof(ubx_set_nav_automotive)))
    {
        fprintf(stderr, "Failed to set automotive model\n");
        close(fd);
        return 1;
    }

    if(verbose) printf(" - Enabling NAV-PVT..\n");
    if(0 != send_ubx_wait_ack(fd, ubx_enable_nav_pvt, sizeof(ubx_enable_nav_pvt)))
    {
        fprintf(stderr, "Failed to enable NAV PVT\n");
        close(fd);
        return 1;
    }

    if(verbose) printf(" - Enabling NAV-SAT..\n");
    if(0 != send_ubx_wait_ack(fd, ubx_enable_nav_sat, sizeof(ubx_enable_nav_sat)))
    {
        fprintf(stderr, "Failed to enable NAV SAT\n");
        close(fd);
        return 1;
    }

    if(verbose) printf(" - Enabling NAV-SIG..\n");
    if(0 != send_ubx_wait_ack(fd, ubx_enable_nav_sig, sizeof(ubx_enable_nav_sig)))
    {
        fprintf(stderr, "Failed to enable NAV SIG\n");
        close(fd);
        return 1;
    }

    if(verbose) printf(" - Enabling MON-RF..\n");
    if(0 != send_ubx_wait_ack(fd, ubx_enable_mon_rf, sizeof(ubx_enable_mon_rf)))
    {
        fprintf(stderr, "Failed to enable MON RF\n");
        close(fd);
        return 1;
    }

    if(verbose) printf(" - Enabling MON-SPAN..\n");
    if(0 != send_ubx_wait_ack(fd, ubx_enable_mon_span, sizeof(ubx_enable_mon_span)))
    {
        fprintf(stderr, "Failed to enable MON SPAN\n");
        close(fd);
        return 1;
    }

    /* SBAS */
    /* Defaults to using SBAS for navigation and differential correction */

    printf("Configuration Successful\n");

    if(udp_host == NULL)
    {
        udp_host = strdup("localhost");
    }

    jammon_datapoint_t jammon_datapoint = { 0 };
    jammon_datapoint.multiband = multiband;

    uint32_t length;
    uint64_t received_monotonic_ms = 0;
    uint64_t last_sent_monotonic_ms = 0;

    while(!app_exit)
    {
        length = wait_ubx(fd, buffer);
        if(length > 0)
        {
            received_monotonic_ms = monotonic_ms();

            #if 0
            for(unsigned int i = 0; i < length; i++)
            {
                printf("%02x", buffer[i]);
            }
            printf("\n");
            #endif

            if(buffer[2] == 0x0a && buffer[3] == 0x38) /* MON-RF */
            {
                if(verbose)
                {
                    printf("# Got MON-RF at %.3f (monotonic)\n", (double)received_monotonic_ms / 1000);
                }

                mon_rf_header_t *rf_header = (mon_rf_header_t *)(&buffer[6]);

                if(rf_header->version != 0x00)
                {
                    fprintf(stderr, "Error: Version mismatch of MON-RF, expected 0x00, received: 0x%02"PRIx8"\n", rf_header->version);
                    continue;
                }

                if(multiband == false)
                {
                    /* Single-band, (probably L1) - eg. M9 */

                    if(rf_header->num_rfblocks != 1)
                    {
                        fprintf(stderr, "Error: Number of MON-RF RF Blocks, expected 1 (single-band), received: %"PRIu8"\n", rf_header->num_rfblocks);
                        continue;
                    }

                    mon_rf_rfblock_t *rf_rfblock = (mon_rf_rfblock_t *)(&buffer[6+4+0]);

                    jammon_datapoint.agc = rf_rfblock->agcCnt;
                    jammon_datapoint.noise = rf_rfblock->noisePerMS;
                    jammon_datapoint.jam_cw = rf_rfblock->jamInd;
                    jammon_datapoint.jam_bb = (rf_rfblock->flags & 0x03) >> 2; /* 0 - unknown, 1 - OK, 2 - Warning, 3 - Critical */
                }
                else
                {
                    /* Dual-band (probably L1 + L2) - eg. F9 */

                    if(rf_header->num_rfblocks != 2)
                    {
                        fprintf(stderr, "Error: Number of MON-RF RF Blocks, expected 2 (multi-band), received: %"PRIu8"\n", rf_header->num_rfblocks);
                        continue;
                    }

                    mon_rf_rfblock_t *rf_rfblock = (mon_rf_rfblock_t *)(&buffer[6+4+0]);

                    jammon_datapoint.agc = rf_rfblock->agcCnt;
                    jammon_datapoint.noise = rf_rfblock->noisePerMS;
                    jammon_datapoint.jam_cw = rf_rfblock->jamInd;
                    jammon_datapoint.jam_bb = (rf_rfblock->flags & 0x03); /* 0 - unknown, 1 - OK, 2 - Warning, 3 - Critical */

                    /* Re-use pointer for second block */
                    rf_rfblock = (mon_rf_rfblock_t *)(&buffer[6+4+24]);

                    jammon_datapoint.agc2 = rf_rfblock->agcCnt;
                    jammon_datapoint.noise2 = rf_rfblock->noisePerMS;
                    jammon_datapoint.jam_cw2 = rf_rfblock->jamInd;
                    jammon_datapoint.jam_bb2 = (rf_rfblock->flags & 0x03); /* 0 - unknown, 1 - OK, 2 - Warning, 3 - Critical */
                }

                jammon_datapoint.mon_rf_monotonic = received_monotonic_ms;
            }
            else if(buffer[2] == 0x0a && buffer[3] == 0x31) /* MON-SPAN */
            {
                if(verbose)
                {
                    printf("# Got MON-SPAN at %.3f (monotonic)\n", (double)received_monotonic_ms / 1000);
                }

                mon_span_header_t *span_header = (mon_span_header_t *)(&buffer[6]);

                if(span_header->version != 0x00)
                {
                    fprintf(stderr, "Error: Version mismatch of MON-SPAN, expected 0x00, received: 0x%02"PRIx8"\n", span_header->version);
                    continue;
                }

                if(multiband == false)
                {
                    /* Single-band, (probably L1) - eg. M9 */

                    if(span_header->num_rfblocks != 1)
                    {
                        fprintf(stderr, "Error: Number of MON-SPAN RF Blocks, expected 1 (single-band), received: %"PRIu8"\n", span_header->num_rfblocks);
                        continue;
                    }

                    mon_span_rfblock_t *span_rfblock = (mon_span_rfblock_t *)(&buffer[6+4+0]);

                    memcpy(jammon_datapoint.spectrum, span_rfblock->spectrum, 256);

                    jammon_datapoint.span = span_rfblock->span;
                    jammon_datapoint.res = span_rfblock->res;
                    jammon_datapoint.center = span_rfblock->center;
                    jammon_datapoint.pga = span_rfblock->pga;
                }
                else
                {
                    /* Dual-band (probably L1 + L2) - eg. F9 */

                    if(span_header->num_rfblocks != 2)
                    {
                        fprintf(stderr, "Error: Number of MON-SPAN RF Blocks, expected 2 (multi-band), received: %"PRIu8"\n", span_header->num_rfblocks);
                        continue;
                    }

                    mon_span_rfblock_t *span_rfblock = (mon_span_rfblock_t *)(&buffer[6+4+0]);

                    memcpy(jammon_datapoint.spectrum, span_rfblock->spectrum, 256);

                    jammon_datapoint.span = span_rfblock->span;
                    jammon_datapoint.res = span_rfblock->res;
                    jammon_datapoint.center = span_rfblock->center;
                    jammon_datapoint.pga = span_rfblock->pga;

                    /* Re-use pointer for second block */
                    span_rfblock = (mon_span_rfblock_t *)(&buffer[6+4+0+272]);

                    memcpy(jammon_datapoint.spectrum2, span_rfblock->spectrum, 256);

                    jammon_datapoint.span2 = span_rfblock->span;
                    jammon_datapoint.res2 = span_rfblock->res;
                    jammon_datapoint.center2 = span_rfblock->center;
                    jammon_datapoint.pga2 = span_rfblock->pga;
                }

                jammon_datapoint.mon_span_monotonic = received_monotonic_ms;
                
            }
            else if(buffer[2] == 0x01 && buffer[3] == 0x07) /* NAV-PVT */
            {
                if(verbose)
                {
                    printf("# Got NAV-PVT at %.3f (monotonic)\n", (double)received_monotonic_ms / 1000);
                }

                nav_pvt_t *pvt = (nav_pvt_t *)(&buffer[6]);

                struct tm tm;
                memset(&tm, 0, sizeof(tm));
                tm.tm_year = pvt->year - 1900;
                tm.tm_mon = pvt->month - 1;
                tm.tm_mday = pvt->day;
                tm.tm_hour = pvt->hour;
                tm.tm_min = pvt->min;
                tm.tm_sec = pvt->sec;

                jammon_datapoint.time_valid = !!((pvt->valid & 0x03) == 0x03);
                jammon_datapoint.gnss_timestamp = mktime(&tm);
                jammon_datapoint.lat = pvt->lat;
                jammon_datapoint.lon = pvt->lon;
                jammon_datapoint.alt = pvt->height;
                jammon_datapoint.h_acc = pvt->hAcc;
                jammon_datapoint.v_acc = pvt->vAcc;

                jammon_datapoint.nav_pvt_monotonic = received_monotonic_ms;
            }
            else if(buffer[2] == 0x01 && buffer[3] == 0x35) /* NAV-SAT */
            {
                if(verbose)
                {
                    printf("# Got NAV-SAT at %.3f (monotonic)\n", (double)received_monotonic_ms / 1000);
                }

                nav_sat_header_t *sat_header = (nav_sat_header_t *)(&buffer[6]);

                jammon_datapoint.svs_nav = 0;

                nav_sat_sv_t *sat_sv;
                for(int i = 0; i < sat_header->num_svs; i++)
                {
                    sat_sv = (nav_sat_sv_t *)(&buffer[6+8+(12*i)]);

                    if(((sat_sv->flags & 0x8) >> 3) == 1)
                    {
                        /* Used in navigation solution */
                        jammon_datapoint.svs_nav++;
                    }
                }

                jammon_datapoint.nav_sat_monotonic = received_monotonic_ms;
            }
            else if(buffer[2] == 0x01 && buffer[3] == 0x43) /* NAV-SIG */
            {
                if(verbose)
                {
                    printf("# Got NAV-SIG at %.3f (monotonic)\n", (double)received_monotonic_ms / 1000);
                }

                nav_sig_header_t *sig_header = (nav_sig_header_t *)(&buffer[6]);

                jammon_datapoint.svs_acquired_l1 = 0;
                jammon_datapoint.svs_acquired_l2 = 0;
                jammon_datapoint.svs_locked_l1 = 0;
                jammon_datapoint.svs_locked_l2 = 0;

                nav_sig_sv_t *sig_sv;
                for(int i = 0; i < sig_header->num_svs; i++)
                {
                    sig_sv = (nav_sig_sv_t *)(&buffer[6+8+(16*i)]);

                    if(sig_sv->qualInd >= 2)
                    {
                        /* SV acquired (note: locked are included) */
                        if(sig_sv->sig_id == 0 || sig_sv->sig_id == 1)
                        {
                            jammon_datapoint.svs_acquired_l1++;
                        }
                        else
                        {
                            jammon_datapoint.svs_acquired_l2++;
                        }
                    }

                    if(sig_sv->qualInd >= 4)
                    {
                        /* SV locked */
                        if(sig_sv->sig_id == 0 || sig_sv->sig_id == 1)
                        {
                            jammon_datapoint.svs_locked_l1++;
                        }
                        else
                        {
                            jammon_datapoint.svs_locked_l2++;
                        }
                    }
                }

                jammon_datapoint.nav_sig_monotonic = received_monotonic_ms;
            }

            if(    (last_sent_monotonic_ms == 0 || last_sent_monotonic_ms + 900 < received_monotonic_ms)
                && (jammon_datapoint.mon_rf_monotonic + 900 > received_monotonic_ms)
                && (jammon_datapoint.mon_span_monotonic + 900 > received_monotonic_ms)
                && (jammon_datapoint.nav_pvt_monotonic + 900 > received_monotonic_ms)
                && (jammon_datapoint.nav_sat_monotonic + 900 > received_monotonic_ms)
                && (jammon_datapoint.nav_sig_monotonic + 900 > received_monotonic_ms)
            )
            {
                process_datapoint(jammon_datapoint, udp_host, udp_port);
                last_sent_monotonic_ms = received_monotonic_ms;
            }
        }
    }

    printf("Received signal, closing..\n");
    close(fd);

    free(udp_host);
   
    return 0;
}
