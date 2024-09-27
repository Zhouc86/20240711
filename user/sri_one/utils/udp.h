//
// Created by han on 24-9-15.
//

#ifndef UDP_H
#define UDP_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <sys/socket.h>
#include <stdint.h>

#define STATE_OUT_T_PACKED_LEN 416
#define PD_IN_T_PACKED_LEN 480

    union byte8 {
        double fdata;
        uint8_t byte[8];
    };

    typedef struct {
        double torque[12];
        double pTarget[12];
        double dTarget[12];
        double pGain[12];
        double dGain[12];
    } pd_motor_in_t;

    typedef struct {
        pd_motor_in_t motor_cmd_data;
    } sri_in_t;

    typedef struct {
        double pos[12];
        double vel[12];
        double torque[12];
    } motor_out_t;

    typedef struct {
        double base_vel[3];
        double base_acc[3];
        double base_rot_vel[3];
        double base_xyz[3];
        double base_orient[4];
    } robot_state_t;

    typedef struct {
        motor_out_t motor_info_data;
        robot_state_t robot_state_data;
    } sri_out_t;

    // Create a UDP socket listening at a specific address/port
    int udp_init_host(const char *addr_str, const char *port_str);

    // Create a UDP socket connected and listening to specific addresses/ports
    int udp_init_client(const char *remote_addr_str, const char *remote_port_str,
                        const char *local_addr_str, const char *local_port_str);

    // Close a UDP socket
    void udp_close(int sock);

    ssize_t get_newest_packet_ros2(int sock, void *recvbuf, struct sockaddr *src_addr, socklen_t *addrlen);

    // Send a packet
    ssize_t send_packet(int sock, void *sendbuf, size_t sendlen,
                        struct sockaddr *dst_addr, socklen_t addrlen);

    void pack_pd_in_t(sri_in_t *bus, unsigned char *bytes);

    void unpack_state_out_t(const unsigned char *bytes, sri_out_t *bus);

#ifdef __cplusplus
}
#endif


#endif //UDP_H
