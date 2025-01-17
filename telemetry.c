#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "main.h"
#include "cmp.h"

#define CMP_BUFFER_SIZE     4096
static uint8_t buffer[CMP_BUFFER_SIZE];

static uint32_t cmp_buf_ptr = 0;

static void udp_send(char *host, uint16_t port, uint8_t *buffer, size_t buffer_size)
{
    int sockfd, n;
    int serverlen;
    struct sockaddr_in serveraddr;

    bzero((char *)&serveraddr, sizeof(serveraddr));

    /* socket: create the socket */
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        fprintf(stderr, "Error: opening socket\n");
        return;
    }

    int result;
    struct addrinfo hints;
    struct addrinfo* pResultList = NULL;

    bzero((char *)&hints, sizeof(struct addrinfo));

    //hints.ai_family = AF_INET; // Use IPv4
    //hints.ai_socktype = SOCK_DGRAM;
    //hints.ai_socktype = SOCK_STREAM;

    result = getaddrinfo(host , NULL, &hints, &pResultList);

    if(result != 0)
    {
        //fprintf(stderr, "Error: Hostname lookup failed for %s, message: %s\n", host, gai_strerror(result));
        close(sockfd);
        return;
    }
    memcpy(&serveraddr, pResultList->ai_addr, sizeof(serveraddr));
    freeaddrinfo(pResultList);

    serveraddr.sin_port = htons(port);

    /* send the message to the server */
    serverlen = sizeof(serveraddr);
    n = sendto(sockfd, buffer, buffer_size, 0, &serveraddr, serverlen);
    if (n < 0)
    {
        fprintf(stderr, "Error: in sendto\n");
    }

    close(sockfd);
}

static bool file_skipper(cmp_ctx_t *ctx, size_t count)
{
    //return fseek((FILE *)ctx->buf, count, SEEK_CUR);
    (void)ctx;
    cmp_buf_ptr += count;
    return true;
}

static size_t file_writer(cmp_ctx_t *ctx, const void *buffer, size_t count) {

    uint16_t i;
    uint8_t *buffer_ptr = (uint8_t *)buffer;

    if (cmp_buf_ptr+count > CMP_BUFFER_SIZE)
    {
        return -1;
    }

    for (i = 0; i < count; i++)
    {
        ((char*)ctx->buf)[cmp_buf_ptr] = *((uint8_t*)buffer_ptr);
        buffer_ptr++;
        cmp_buf_ptr++;
    }
    return count;
    //return fwrite(data, sizeof(uint8_t), count, (FILE *)ctx->buf);
}

void udp_send_msgpack(char *host, uint16_t port, jammon_datapoint_t *jammon_datapoint_ptr)
{
    cmp_ctx_t cmp;

    cmp_buf_ptr = 0;
    cmp_init(&cmp, (void*)buffer, 0, file_skipper, file_writer);

    /* Start map, 7 items, 8 items if multiband (spectrum2) */
    cmp_write_map(&cmp, (jammon_datapoint_ptr->multiband ? 10 : 7));

    /* GNSS timestamp */
    cmp_write_uint(&cmp, 0);
    cmp_write_uint(&cmp, jammon_datapoint_ptr->gnss_timestamp);

    /* Array of [lat, lon, alt] */
    cmp_write_uint(&cmp, 1);
    cmp_write_array(&cmp, 3);
    cmp_write_sint(&cmp, jammon_datapoint_ptr->lat);
    cmp_write_sint(&cmp, jammon_datapoint_ptr->lon);
    cmp_write_sint(&cmp, jammon_datapoint_ptr->alt);

    /* Array of [hAcc, vAcc] */
    cmp_write_uint(&cmp, 2);
    cmp_write_array(&cmp, 2);
    cmp_write_uint(&cmp, jammon_datapoint_ptr->h_acc);
    cmp_write_uint(&cmp, jammon_datapoint_ptr->v_acc);

    /* Array of [svs_acquired, svs_locked, svs_nav] */
    cmp_write_uint(&cmp, 3);
    cmp_write_array(&cmp, 5);
    cmp_write_uint(&cmp, jammon_datapoint_ptr->svs_acquired_l1);
    cmp_write_uint(&cmp, jammon_datapoint_ptr->svs_acquired_l2);
    cmp_write_uint(&cmp, jammon_datapoint_ptr->svs_locked_l1);
    cmp_write_uint(&cmp, jammon_datapoint_ptr->svs_locked_l2);
    cmp_write_uint(&cmp, jammon_datapoint_ptr->svs_nav);

    /* Array of [agc, noise] */
    cmp_write_uint(&cmp, 4);
    cmp_write_array(&cmp, 2);
    cmp_write_uint(&cmp, jammon_datapoint_ptr->agc);
    cmp_write_uint(&cmp, jammon_datapoint_ptr->noise);

    /* Array of [jam_cw, jam_bb] */
    cmp_write_uint(&cmp, 5);
    cmp_write_array(&cmp, 2);
    cmp_write_uint(&cmp, jammon_datapoint_ptr->jam_cw);
    cmp_write_uint(&cmp, jammon_datapoint_ptr->jam_bb);

    if(jammon_datapoint_ptr->multiband)
    {
        /* Array of [agc, noise] */
        cmp_write_uint(&cmp, 6);
        cmp_write_array(&cmp, 2);
        cmp_write_uint(&cmp, jammon_datapoint_ptr->agc2);
        cmp_write_uint(&cmp, jammon_datapoint_ptr->noise2);

        /* Array of [jam_cw, jam_bb] */
        cmp_write_uint(&cmp, 7);
        cmp_write_array(&cmp, 2);
        cmp_write_uint(&cmp, jammon_datapoint_ptr->jam_cw2);
        cmp_write_uint(&cmp, jammon_datapoint_ptr->jam_bb2);
    }

    /* Array of [center, res, spectrum[256], pga] */
    cmp_write_uint(&cmp, 10);
    cmp_write_array(&cmp, 4);
    cmp_write_uint(&cmp, jammon_datapoint_ptr->center);
    cmp_write_uint(&cmp, jammon_datapoint_ptr->res);
    cmp_write_bin(&cmp, jammon_datapoint_ptr->spectrum, 256);
    cmp_write_uint(&cmp, jammon_datapoint_ptr->pga);

    if(jammon_datapoint_ptr->multiband)
    {
        /* Array of [center, res, spectrum[256], pga] */
        cmp_write_uint(&cmp, 11);
        cmp_write_array(&cmp, 4);
        cmp_write_uint(&cmp, jammon_datapoint_ptr->center2);
        cmp_write_uint(&cmp, jammon_datapoint_ptr->res2);
        cmp_write_bin(&cmp, jammon_datapoint_ptr->spectrum2, 256);
        cmp_write_uint(&cmp, jammon_datapoint_ptr->pga2);
    }

    udp_send(host, port, buffer, cmp_buf_ptr);
}