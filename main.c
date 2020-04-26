#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>

#include "main.h"
#include "telemetry.h"

static bool app_exit;
static bool verbose = false;

/*** Disable NMEA ***/
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

/*** MODEL, 4 = automotive ***/
/* UBX-CFG-NAV5 */
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

/* Time & Position */
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

typedef struct {
    uint32_t pinSel, pinBank, pinDir, pinVal;

    uint16_t noisePerMS;
    uint16_t agcCnt;

    uint8_t aStatus;
    uint8_t aPower;
    uint8_t flags;
    uint8_t res1;

    uint32_t usedMask;

    uint8_t VP[17];
    uint8_t jamInd; /* CW jamming measure */
    uint16_t res2;
    uint32_t pinIrq, pullH, pullL;
} __attribute__((packed)) mon_hw_t;

/* Jamming indicators */
static const uint8_t ubx_enable_mon_hw[] = {
    0xb5, 0x62,
    0x06, 0x01, /* UBX-CFG-MSG */
    0x08, 0x00,
    0x0A, 0x09, /* Enable UBX-MON-HW */
    0x00, /* Port 0 (I2C) */
    0x00, /* Port 1 (UART/UART1) */
    0x00, /* Port 2 (UART2) */
    0x01, /* Port 3 (USB) at 1 second interval */
    0x00, /* Port 4 */
    0x00, /* Port 5 */
    0x23, 0x35 /* Checksum */
};

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

/* Enable Interference Detection, BB threshold = 3dB, CW = 15dB, Active antenna */
static const uint8_t ubx_enable_itfm[] = {
    0xb5, 0x62,
    0x06, 0x39, /* UBX-CFG-ITFM */
    0x08, 0x00,
    0xf3, 0xac, 0x62, 0xad, 0x1e, 0x63, 0x31, 0x00,
    0xa7, 0x07 /* Checksum */
};

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

static uint64_t monotonic_ms(void)
{
    struct timespec tp;

    if(clock_gettime(CLOCK_MONOTONIC, &tp) != 0)
    {
        return 0;
    }

    return (uint64_t) tp.tv_sec * 1000 + tp.tv_nsec / 1000000;
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
    uint8_t msg_class_id = buffer[2];
    uint8_t msg_msg_id = buffer[3];

    tcflush(fd, TCIFLUSH);
    if(write(fd, buffer, buffer_size) != (int)buffer_size)
    {
        return 3;
    }

    uint8_t response_byte;
    uint32_t response_index = 0;
    for(k = 0; (k < 1000) && !app_exit; k++)
    {
        if(read(fd, &response_byte, 1) > 0)
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

static const uint8_t msg_header[2] = { 0xb5, 0x62 };
static uint32_t wait_ubx(int fd, uint8_t *buffer)
{
    uint8_t response_byte;
    uint32_t response_index = 0;
    uint32_t response_length = 0;

    while(!app_exit)
    {
        if(read(fd, &response_byte, 1) > 0)
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

void sigint_handler(int sig)
{
    (void)sig;
    app_exit = true;
}

static void usage( void )
{
    printf("Usage: jammon [-v] -d <device>\n");  
}
 
int main(int argc, char *argv[])
{
    int fd;
    int option = 0;
    char devName[50];
    struct termios tty;

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);
   
    memset(devName, 0, sizeof(devName));
    while((option = getopt( argc, argv, "vd:")) != -1)
    {
        switch(option)
        {
            case 'd':
                printf(" * Using serial device: %s\n", optarg);
                strncpy(devName, optarg, sizeof(devName)-1);                
                break;
            case 'v':
                printf(" * Verbose Mode Enabled\n");
                verbose = true;
                break;
            default:
                usage();
                return 0;
        }        
    }
 
    if(devName[0] == 0)
    {
        usage();
        return 1;
    }
       
    /* Open Serial Port */
    fd = open(devName, O_RDWR);
    if(fd < 0)
    {
        fprintf(stderr, "Error: Cannot open serial device '%s'\n", devName);
        return 1;
    }

    if (tcgetattr (fd, &tty) != 0)
    {
        fprintf(stderr, "Error: tcgetattr\n");
        return -1;
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

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
            fprintf(stderr, "Error: tcsetattr\n");
            return -1;
    }

    printf("Configuring..\n");

    if(0 != send_ubx_wait_ack(fd, ubx_disable_nmea_usb, sizeof(ubx_disable_nmea_usb)))
    {
        fprintf(stderr, "Failed to disable NMEA on USB\n");
        //close(fd);
        //return 1;
    }

    if(0 != send_ubx_wait_ack(fd, ubx_enable_nav_pvt, sizeof(ubx_enable_nav_pvt)))
    {
        fprintf(stderr, "Failed to enable NAV PVT\n");
        //close(fd);
        //return 1;
    }

    if(0 != send_ubx_wait_ack(fd, ubx_enable_nav_sat, sizeof(ubx_enable_nav_sat)))
    {
        fprintf(stderr, "Failed to enable NAV SAT\n");
        //close(fd);
        //return 1;
    }

    if(0 != send_ubx_wait_ack(fd, ubx_enable_mon_hw, sizeof(ubx_enable_mon_hw)))
    {
        fprintf(stderr, "Failed to enable MON HW\n");
        //close(fd);
        //return 1;
    }

    if(0 != send_ubx_wait_ack(fd, ubx_enable_mon_span, sizeof(ubx_enable_mon_span)))
    {
        fprintf(stderr, "Failed to enable MON SPAN\n");
        //close(fd);
        //return 1;
    }

    if(0 != send_ubx_wait_ack(fd, ubx_enable_itfm, sizeof(ubx_enable_itfm)))
    {
        fprintf(stderr, "Failed to enable interference detection\n");
        //close(fd);
        //return 1;
    }

    if(0 != send_ubx_wait_ack(fd, ubx_set_nav_automotive, sizeof(ubx_set_nav_automotive)))
    {
        fprintf(stderr, "Failed to set automotive model\n");
        //close(fd);
        //return 1;
    }

    printf("Configuration Successful\n");

    jammon_datapoint_t jammon_datapoint = { 0 };

    uint32_t length;
    uint8_t buffer[1024];

    FILE *csv_fptr;
    char csv_filename[32];

    while(!app_exit)
    {
        length = wait_ubx(fd, buffer);
        if(length > 0)
        {
            #if 0
            for(unsigned int i = 0; i < length; i++)
            {
                printf("%02x", buffer[i]);
            }
            printf("\n");
            #endif

            /* Order that we're expecting is, within ~100ms of each other:
                1. MON-HW
                2. MON-SPAN
                3. NAV-PVT
                4. NAV-SAT
            */

            if(buffer[2] == 0x0a && buffer[3] == 0x09) /* MON-HW */
            {
                if(verbose)
                {
                    printf("# Got MON-HW at %.3f (monotonic)\n", (double)monotonic_ms() / 1000);
                }

                mon_hw_t *hw = (mon_hw_t *)(&buffer[6]);

                jammon_datapoint.agc = hw->agcCnt;
                jammon_datapoint.noise = hw->noisePerMS;
                jammon_datapoint.jam_cw = hw->jamInd;
                jammon_datapoint.jam_bb = (hw->flags & 0x0c) >> 2; /* 0 - unknown, 1 - OK, 2 - Warning, 3 - Critical */

                jammon_datapoint.mon_hw_monotonic = monotonic_ms();
            }
            else if(buffer[2] == 0x0a && buffer[3] == 0x31) /* MON-SPAN */
            {
                if(verbose)
                {
                    printf("# Got MON-SPAN at %.3f (monotonic)\n", (double)monotonic_ms() / 1000);
                }

                mon_span_header_t *span_header = (mon_span_header_t *)(&buffer[6]);

                if(span_header->version != 0x00)
                {
                    fprintf(stderr, "Error: Version mismatch of MON-SPAN, expected 0x00, received: 0x%02x\n", span_header->version);
                    continue;
                }

                if(span_header->num_rfblocks != 1)
                {
                    fprintf(stderr, "Error: Number of MON-SPAN RF Blocks, expected 1, received: %d\n", span_header->num_rfblocks);
                    continue;
                }

                /* Assume we only want first rfblock */
                mon_span_rfblock_t *span_rfblock = (mon_span_rfblock_t *)(&buffer[6+4+0]);

                memcpy(jammon_datapoint.spectrum, span_rfblock->spectrum, 256);

                jammon_datapoint.span = span_rfblock->span;
                jammon_datapoint.res = span_rfblock->res;
                jammon_datapoint.center = span_rfblock->center;
                jammon_datapoint.pga = span_rfblock->pga;

                jammon_datapoint.mon_span_monotonic = monotonic_ms();
            }
            else if(buffer[2] == 0x01 && buffer[3] == 0x07) /* NAV-PVT */
            {
                if(verbose)
                {
                    printf("# Got NAV-PVT at %.3f (monotonic)\n", (double)monotonic_ms() / 1000);
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

                jammon_datapoint.nav_pvt_monotonic = monotonic_ms();
            }
            else if(buffer[2] == 0x01 && buffer[3] == 0x35) /* NAV-SAT */
            {
                if(verbose)
                {
                    printf("# Got NAV-SAT at %.3f (monotonic)\n", (double)monotonic_ms() / 1000);
                }

                nav_sat_header_t *sat_header = (nav_sat_header_t *)(&buffer[6]);

                jammon_datapoint.svs_acquired = 0;
                jammon_datapoint.svs_locked = 0;
                jammon_datapoint.svs_nav = 0;

                nav_sat_sv_t *sat_sv;
                for(int i = 0; i < sat_header->num_svs; i++)
                {
                    sat_sv = (nav_sat_sv_t *)(&buffer[6+8+(12*i)]);

                    if((sat_sv->flags & 0x7) >= 2)
                    {
                        /* SV acquired */
                        jammon_datapoint.svs_acquired++;
                    }

                    if((sat_sv->flags & 0x7) >= 4)
                    {
                        /* SV locked (note: acquired are included) */
                        jammon_datapoint.svs_locked++;
                    }

                    if(((sat_sv->flags & 0x8) >> 3) == 1)
                    {
                        /* Used in navigation solution */
                        jammon_datapoint.svs_nav++;
                    }
                }

                /* Only log if we've got a valid time, all data is populated, and less than 700 milliseconds old */
                uint64_t now_monotonic = monotonic_ms();
                if(jammon_datapoint.time_valid
                    && (jammon_datapoint.mon_hw_monotonic > 0) && (jammon_datapoint.mon_hw_monotonic + 700 > now_monotonic)
                    && (jammon_datapoint.mon_span_monotonic > 0) && (jammon_datapoint.mon_span_monotonic + 700 > now_monotonic)
                    && (jammon_datapoint.nav_pvt_monotonic > 0) && (jammon_datapoint.nav_pvt_monotonic + 700 > now_monotonic)
                )
                {
                    if(verbose)
                    {
                        printf("Datapoint:\n");
                        printf(" - Timestamp: %lld - %.24s\n", jammon_datapoint.gnss_timestamp, ctime((time_t *)&jammon_datapoint.gnss_timestamp));
                        printf(" - Position: %d, %d, %d\n", jammon_datapoint.lat, jammon_datapoint.lon, jammon_datapoint.alt);
                        printf(" - Accuracy: H: %.1fm, V: %.1fm\n", jammon_datapoint.h_acc / 1.0e3, jammon_datapoint.v_acc / 1.0e3);
                        printf(" - SVs: Acquired: %d, Locked: %d, Used in Nav: %d\n", jammon_datapoint.svs_acquired, jammon_datapoint.svs_locked, jammon_datapoint.svs_nav);
                        printf(" - AGC: %d, Noise: %d\n", jammon_datapoint.agc, jammon_datapoint.noise);
                        printf(" - Jamming: CW: %d / 255, Broadband: %d / 3 (0 = invalid)\n", jammon_datapoint.jam_cw, jammon_datapoint.jam_bb);
                    }

                    char *csv_output_line;
                    asprintf(&csv_output_line, "%lld,%.24s,%.5f,%.5f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d\n",
                        jammon_datapoint.gnss_timestamp, ctime((time_t *)&jammon_datapoint.gnss_timestamp),
                        (jammon_datapoint.lat / 1.0e7), (jammon_datapoint.lon / 1.0e7), (jammon_datapoint.alt / 1.0e3),
                        jammon_datapoint.h_acc / 1.0e3, jammon_datapoint.v_acc / 1.0e3,
                        jammon_datapoint.svs_acquired, jammon_datapoint.svs_locked, jammon_datapoint.svs_nav,
                        jammon_datapoint.agc, jammon_datapoint.noise,
                        jammon_datapoint.jam_cw, jammon_datapoint.jam_bb
                    );

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

                    /* Allocate 2*256, + 1 for sprintf null termination */
                    char *csv_output_spectrum = malloc((2*256)+1);

                    if(csv_output_spectrum == NULL)
                    {
                        fprintf(stderr, "Error: Unable to allocate memory for spectrum CSV line\n");
                        continue;
                    }

                    for(int i = 0; i < 255; i++)
                    {
                        sprintf(&csv_output_spectrum[i*2], "%02x", jammon_datapoint.spectrum[i]);
                    }

                    asprintf(&csv_output_line, "%lld,%d,%d,%d,%d,\"%s\"\n",
                        jammon_datapoint.gnss_timestamp,
                        jammon_datapoint.span, jammon_datapoint.res, jammon_datapoint.center, jammon_datapoint.pga,
                        csv_output_spectrum
                    );
                    free(csv_output_spectrum);

                    strftime(csv_filename, 31, "spectrum-jammon-%Y-%m-%d.csv", localtime((time_t *)&(jammon_datapoint.gnss_timestamp)));

                    csv_fptr = fopen(csv_filename, "a+"); 
                    if(csv_fptr != NULL)
                    {
                        fputs(csv_output_line, csv_fptr);
                        fclose(csv_fptr);
                    }
                    else
                    {
                        fprintf(stderr, "Error: Unable to open spectrum CSV file\n");
                    }
                    free(csv_output_line);

                    /* Lastly, send UDP Telemetry */
                    udp_send_msgpack(&jammon_datapoint);
                }
            }
        }

    }

    printf("Received signal, closing..\n");
    close(fd);
   
    return 0;
}
