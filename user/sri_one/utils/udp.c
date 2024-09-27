//
// Created by han on 24-9-15.
//

#include "udp.h"

#include <sys/ioctl.h>
#include <poll.h>
#include <netdb.h>
#include <unistd.h>
#define SOCKETS_INIT
#define SOCKETS_CLEANUP
typedef int ioctl_arg_t;

int udp_init_host(const char *local_addr_str, const char *local_port_str) {
    // Platform-specific socket library initialization
    SOCKETS_INIT;

    int err;

    // Get address info
    struct addrinfo *local;
    struct addrinfo hints = {0};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    err = getaddrinfo(local_addr_str, local_port_str, &hints, &local);
    if (err) {
        printf("%s\n", gai_strerror(err));
        SOCKETS_CLEANUP;
        return -1;
    }

    // Create socket
    int sock = socket(local->ai_family, local->ai_socktype, local->ai_protocol);
    if (-1 == sock) {
        perror("Error creating socket");
        freeaddrinfo(local);
        SOCKETS_CLEANUP;
        return -1;
    }

    // Bind to interface address
    if (bind(sock, (struct sockaddr *) local->ai_addr,
             local->ai_addrlen)) {
        perror("Error binding to interface address");
        close(sock);
        freeaddrinfo(local);
        SOCKETS_CLEANUP;
        return -1;
    }

    // Free addrinfo struct
    freeaddrinfo(local);

    // Make socket non-blocking
    ioctl_arg_t mode = 1;
    ioctl(sock, FIONBIO, &mode);

    return sock;
}

int udp_init_client(const char *remote_addr_str, const char *remote_port_str,
                    const char *local_addr_str, const char *local_port_str) {
    // Platform-specific socket library initialization
    SOCKETS_INIT;

    int err;

    // Get remote address info
    struct addrinfo *remote;
    struct addrinfo hints = {0};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    err = getaddrinfo(remote_addr_str, remote_port_str, &hints, &remote);
    if (err) {
        printf("%s\n", gai_strerror(err));
        SOCKETS_CLEANUP;
        return -1;
    }

    // Get remote address info
    struct addrinfo *local;
    err = getaddrinfo(local_addr_str, local_port_str, &hints, &local);
    if (err) {
        printf("%s\n", gai_strerror(err));
        SOCKETS_CLEANUP;
        return -1;
    }

    // Create socket
    int sock = socket(remote->ai_family, remote->ai_socktype,
                      remote->ai_protocol);
    if (-1 == sock) {
        perror("Error creating socket");
        freeaddrinfo(remote);
        freeaddrinfo(local);
        SOCKETS_CLEANUP;
        return -1;
    }

    // Bind to interface address
    if (bind(sock, (struct sockaddr *) local->ai_addr, local->ai_addrlen)) {
        perror("Error binding to interface address");
        close(sock);
        freeaddrinfo(remote);
        freeaddrinfo(local);
        SOCKETS_CLEANUP;
        return -1;
    }

    // Connect to remote address
    if (connect(sock, (struct sockaddr *) remote->ai_addr,
                remote->ai_addrlen)) {
        perror("Error connecting to remote address");
        close(sock);
        freeaddrinfo(remote);
        freeaddrinfo(local);
        SOCKETS_CLEANUP;
        return -1;
    }

    // Free addrinfo structs
    freeaddrinfo(remote);
    freeaddrinfo(local);

    // Make socket non-blocking
    ioctl_arg_t mode = 1;
    ioctl(sock, FIONBIO, &mode);

    return sock;
}

void udp_close(int sock) {
    close(sock);
    SOCKETS_CLEANUP;
}

ssize_t send_packet(int sock, void *sendbuf, size_t sendlen,
                    struct sockaddr *dst_addr, socklen_t addrlen) {
    ssize_t nbytes;

    // Send packet, retrying if busy
    do {
        nbytes = sendto(sock, sendbuf, sendlen, 0, dst_addr, addrlen);
    } while (-1 == nbytes);

    // Return the sent packet size
    return nbytes;
}

void unpack_pd_in_t(const unsigned char *bytes, sri_in_t *bus) {
    union byte8 tmp_data;

    int byte_counter = 0;
    // 1. torque
    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 8; ++j) {
            tmp_data.byte[j] = bytes[byte_counter++];
        }
        bus->motor_cmd_data.torque[i] = tmp_data.fdata;
    }

    // 2. target position
    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 8; ++j) {
            tmp_data.byte[j] = bytes[byte_counter++];
        }
        bus->motor_cmd_data.pTarget[i] = tmp_data.fdata;
    }

    // 3. target vel
    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 8; ++j) {
            tmp_data.byte[j] = bytes[byte_counter++];
        }
        bus->motor_cmd_data.dTarget[i] = tmp_data.fdata;
    }

    // 4. gain P
    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 8; ++j) {
            tmp_data.byte[j] = bytes[byte_counter++];
        }
        bus->motor_cmd_data.pGain[i] = tmp_data.fdata;
    }

    // 5. gain d
    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 8; ++j) {
            tmp_data.byte[j] = bytes[byte_counter++];
        }
        bus->motor_cmd_data.dGain[i] = tmp_data.fdata;
    }
}


void pack_pd_in_t(sri_in_t *bus, unsigned char *bytes) {
    union byte8 tmp_data;

    int byte_counter = 0;
    // 1. torque

    for (int i = 0; i < 12; ++i) {
        tmp_data.fdata = bus->motor_cmd_data.torque[i];
        for (int j = 0; j < 8; ++j) {
            bytes[byte_counter++] = tmp_data.byte[j];
        }
    }

    // 2. target position
    for (int i = 0; i < 12; ++i) {
        tmp_data.fdata = bus->motor_cmd_data.pTarget[i];
        for (int j = 0; j < 8; ++j) {
            bytes[byte_counter++] = tmp_data.byte[j];
        }
    }

    // 3. target vel
    for (int i = 0; i < 12; ++i) {
        tmp_data.fdata = bus->motor_cmd_data.dTarget[i];
        for (int j = 0; j < 8; ++j) {
            bytes[byte_counter++] = tmp_data.byte[j];
        }
    }

    // 4. gain p
    for (int i = 0; i < 12; ++i) {
        tmp_data.fdata = bus->motor_cmd_data.pGain[i];
        for (int j = 0; j < 8; ++j) {
            bytes[byte_counter++] = tmp_data.byte[j];
        }
    }

    // 5. gain d
    for (int i = 0; i < 12; ++i) {
        tmp_data.fdata = bus->motor_cmd_data.dGain[i];
        for (int j = 0; j < 8; ++j) {
            bytes[byte_counter++] = tmp_data.byte[j];
        }
    }
}

void pack_state_out_t(const sri_out_t *bus, unsigned char *bytes) {
    union byte8 tmp_data;
    int byte_counter = 0;

    // 1 position
    for (int i = 0; i < 12; ++i) {
        tmp_data.fdata = bus->motor_info_data.pos[i];
        for (int j = 0; j < 8; ++j) {
            bytes[byte_counter++] = tmp_data.byte[j];
        }
    }

    // 2 velocity
    for (int i = 0; i < 12; ++i) {
        tmp_data.fdata = bus->motor_info_data.vel[i];
        for (int j = 0; j < 8; ++j) {
            bytes[byte_counter++] = tmp_data.byte[j];
        }
    }

    // 3 acc
    for (int i = 0; i < 12; ++i) {
        tmp_data.fdata = bus->motor_info_data.torque[i];
        for (int j = 0; j < 8; ++j) {
            bytes[byte_counter++] = tmp_data.byte[j];
        }
    }


    // 8. base velocity
    for (int i = 0; i < 3; ++i) {
        tmp_data.fdata = bus->robot_state_data.base_vel[i];
        for (int j = 0; j < 8; ++j) {
            bytes[byte_counter++] = tmp_data.byte[j];
        }
    }

    // 9. base acc
    for (int i = 0; i < 3; ++i) {
        tmp_data.fdata = bus->robot_state_data.base_acc[i];
        for (int j = 0; j < 8; ++j) {
            bytes[byte_counter++] = tmp_data.byte[j];
        }
    }

    //10. base rot vel
    for (int i = 0; i < 3; ++i) {
        tmp_data.fdata = bus->robot_state_data.base_rot_vel[i];
        for (int j = 0; j < 8; ++j) {
            bytes[byte_counter++] = tmp_data.byte[j];
        }
    }

    //11. base xyz
    for (int i = 0; i < 3; ++i) {
        tmp_data.fdata = bus->robot_state_data.base_xyz[i];
        for (int j = 0; j < 8; ++j) {
            bytes[byte_counter++] = tmp_data.byte[j];
        }
    }

    //12. base orient
    for (int i = 0; i < 3; ++i) {
        tmp_data.fdata = bus->robot_state_data.base_orient[i];
        for (int j = 0; j < 8; ++j) {
            bytes[byte_counter++] = tmp_data.byte[j];
        }
    }
}

void unpack_state_out_t(const unsigned char *bytes, sri_out_t *bus) {
    union byte8 tmp_data;
    int byte_counter = 0;

    // 1 position
    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 8; ++j) {
            tmp_data.byte[j] = bytes[byte_counter++];
        }
        bus->motor_info_data.pos[i] = tmp_data.fdata;
    }

    // 2 velocity
    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 8; ++j) {
            tmp_data.byte[j] = bytes[byte_counter++];
        }
        bus->motor_info_data.vel[i] = tmp_data.fdata;
    }

    // 3 acc
    for (int i = 0; i < 12; ++i) {
        for (int j = 0; j < 8; ++j) {
            tmp_data.byte[j] = bytes[byte_counter++];
        }
        bus->motor_info_data.torque[i] = tmp_data.fdata;
    }

    // 8. base velocity
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 8; ++j) {
            tmp_data.byte[j] = bytes[byte_counter++];
        }
        bus->robot_state_data.base_vel[i] = tmp_data.fdata;
    }

    // 9. base acc
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 8; ++j) {
            tmp_data.byte[j] = bytes[byte_counter++];
        }
        bus->robot_state_data.base_acc[i] = tmp_data.fdata;
    }

    // 10. base rot vel
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 8; ++j) {
            tmp_data.byte[j] = bytes[byte_counter++];
        }
        bus->robot_state_data.base_rot_vel[i] = tmp_data.fdata;
    }

    // 11. base xyz
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 8; ++j) {
            tmp_data.byte[j] = bytes[byte_counter++];
        }
        bus->robot_state_data.base_xyz[i] = tmp_data.fdata;
    }

    // 12. base orient
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 8; ++j) {
            tmp_data.byte[j] = bytes[byte_counter++];
        }
        bus->robot_state_data.base_orient[i] = tmp_data.fdata;
    }
}

ssize_t get_newest_packet_ros2(int sock, void *recvbuf, struct sockaddr *src_addr, socklen_t *addrlen) {
    // Does not use sequence number for determining newest packet
    ssize_t nbytes = -1;
    struct pollfd fd = {.fd = sock, .events = POLLIN, .revents = 0};

    // Loop through RX buffer, copying data if packet is correct size
    while (poll(&fd, 1, 1)) {
        ioctl_arg_t nbytes_avail;
        ioctl(sock, FIONREAD, &nbytes_avail);
        nbytes = recvfrom(sock, recvbuf, nbytes_avail, 0, src_addr, addrlen);
    }

    // Return the copied packet size, or -1 if no data was copied
    return nbytes;

}