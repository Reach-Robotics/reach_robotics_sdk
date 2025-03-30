#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <ctype.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define DATA_BYTES_PER_PACKET 64
#define RESPONSE_TIMEOUT 0.001

typedef enum {
    FWD_ALLOW = 0b00000001,
    IS_DEMAND = 0b00000010,
    PROTOCOL_V2 = 0b00000100
} PacketOption;

typedef struct {
    uint8_t length;
    uint8_t address;
    uint16_t code;
    uint16_t crc;
    uint8_t data[DATA_BYTES_PER_PACKET];
    uint8_t transmitData[DATA_BYTES_PER_PACKET];
    uint8_t protocol;
    uint8_t option;
    uint8_t useOption;
    uint16_t receiveRegister;
    uint8_t totalFrames;
} Packet_t;

typedef struct {
    int fd;
    struct sockaddr_in address;
    bool is_socket;
} Connection;

void encode_packet(Packet_t *packet, uint8_t device_id, uint16_t packet_id, uint8_t *data, size_t data_len, uint8_t option) {
    packet->address = device_id;
    packet->code = packet_id;
    packet->length = data_len + 4;
    memcpy(packet->data, data, data_len);
    packet->useOption = (option != 0);
    packet->option = option;
    // Add encoding logic here (e.g., CRC calculation, etc.)
}

bool decode_packet(Packet_t *packet, uint8_t *buffer, size_t buffer_len) {
    if (buffer_len < 4) {
        return false;
    }
    memcpy(packet, buffer, sizeof(Packet_t));
    // Add decoding logic here (e.g., CRC validation, etc.)
    return true;
}

Connection *create_serial_connection(const char *serial_port, int baud_rate) {
    Connection *conn = malloc(sizeof(Connection));
    conn->is_socket = false;
    conn->fd = open(serial_port, O_RDWR | O_NOCTTY | O_SYNC);
    if (conn->fd < 0) {
        perror("Error opening serial port");
        free(conn);
        return NULL;
    }

    struct termios tty;
    if (tcgetattr(conn->fd, &tty) != 0) {
        perror("Error getting terminal attributes");
        close(conn->fd);
        free(conn);
        return NULL;
    }

    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.lflag = 0;
    tty.oflag = 0;
    tty.cc[VMIN] = 1;
    tty.cc[VTIME] = 1;

    if (tcsetattr(conn->fd, TCSANOW, &tty) != 0) {
        perror("Error setting terminal attributes");
        close(conn->fd);
        free(conn);
        return NULL;
    }

    return conn;
}

Connection *create_socket_connection(const char *ip, int port) {
    Connection *conn = malloc(sizeof(Connection));
    conn->is_socket = true;
    conn->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (conn->fd < 0) {
        perror("Error creating socket");
        free(conn);
        return NULL;
    }

    memset(&conn->address, 0, sizeof(conn->address));
    conn->address.sin_family = AF_INET;
    conn->address.sin_port = htons(port);
    inet_pton(AF_INET, ip, &conn->address.sin_addr);

    return conn;
}

void write_packet(Connection *conn, Packet_t *packet) {
    if (conn->is_socket) {
        sendto(conn->fd, packet, sizeof(Packet_t), 0, (struct sockaddr *)&conn->address, sizeof(conn->address));
    } else {
        write(conn->fd, packet, sizeof(Packet_t));
    }
}

ssize_t read_packet(Connection *conn, uint8_t *buffer, size_t buffer_len) {
    if (conn->is_socket) {
        socklen_t addr_len = sizeof(conn->address);
        return recvfrom(conn->fd, buffer, buffer_len, 0, (struct sockaddr *)&conn->address, &addr_len);
    } else {
        return read(conn->fd, buffer, buffer_len);
    }
}

void close_connection(Connection *conn) {
    close(conn->fd);
    free(conn);
}

// Add more functions for packet handling, encoding/decoding floats and integers, etc.

///////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <stdbool.h>

#define DATA_BYTES_PER_PACKET 64
#define PKT_HEADER_LEN 4
#define MAX_PACKET_DATA_SIZE 64

typedef struct {
    uint8_t length;
    uint8_t address;
    uint16_t code;
    uint16_t crc;
    uint8_t data[DATA_BYTES_PER_PACKET];
    uint8_t transmitData[DATA_BYTES_PER_PACKET];
    uint8_t protocol;
    uint8_t option;
    uint8_t useOption;
    uint16_t receiveRegister;
    uint8_t totalFrames;
} Packet_t;

void *drivers = NULL;
char *lib_path = NULL;

void import_drivers() {
    printf("IMPORTING RS1 DRIVERS\n");
    drivers = dlopen(lib_path, RTLD_LAZY);
    if (!drivers) {
        fprintf(stderr, "Failed to load driver library: %s\n", dlerror());
        exit(EXIT_FAILURE);
    }
}

uint8_t *encode_packet(int device_id, int packet_id, uint8_t *data, size_t data_len, int *option, size_t *output_len) {
    if (!drivers) {
        import_drivers();
    }

    Packet_t p;
    memset(&p, 0, sizeof(Packet_t));

    p.address = (uint8_t)device_id;
    p.code = (uint16_t)packet_id;
    p.length = (uint8_t)(data_len + PKT_HEADER_LEN);

    if (data_len > DATA_BYTES_PER_PACKET) {
        fprintf(stderr, "encode_packet(): length (%zu) is greater than MAX_PACKET_DATA_SIZE (%d)\n", data_len, DATA_BYTES_PER_PACKET);
        return NULL;
    }

    memcpy(p.data, data, data_len);

    if (option) {
        p.useOption = 1;
        p.option = (uint8_t)(*option);
    } else {
        p.useOption = 0;
        p.option = 0;
    }

    typedef int (*coms_encodePacket_t)(Packet_t *, uint8_t, uint16_t, uint8_t, uint8_t *, uint8_t);
    coms_encodePacket_t coms_encodePacket = (coms_encodePacket_t)dlsym(drivers, "coms_encodePacket");

    if (!coms_encodePacket) {
        fprintf(stderr, "Failed to load coms_encodePacket function: %s\n", dlerror());
        return NULL;
    }

    int ret = coms_encodePacket(&p, p.address, p.code, p.length, p.data, p.option);

    if (ret == 1) {
        *output_len = p.length;
        uint8_t *output = malloc(p.length);
        memcpy(output, p.transmitData, p.length);
        return output;
    } else {
        fprintf(stderr, "encode_packet(): unknown error code %d\n", ret);
        return NULL;
    }
}

int parse_packet(uint8_t *packet_in, size_t packet_len, int *device_id, int *packet_id, uint8_t **data, size_t *data_len, int *option) {
    if (!drivers) {
        import_drivers();
    }

    if (packet_len < PKT_HEADER_LEN) {
        fprintf(stderr, "parse_packet(): packet length is less than PKT_HEADER_LEN (%d)\n", PKT_HEADER_LEN);
        return -1;
    }

    Packet_t p;
    memset(&p, 0, sizeof(Packet_t));

    if (packet_len > DATA_BYTES_PER_PACKET) {
        fprintf(stderr, "parse_packet(): packet_in is too large to parse. packet_len: %zu\n", packet_len);
        return -1;
    }

    memcpy(p.data, packet_in, packet_len);

    typedef int (*coms_decodePacket_t)(Packet_t *, uint8_t *, size_t);
    coms_decodePacket_t coms_decodePacket = (coms_decodePacket_t)dlsym(drivers, "coms_decodePacket");

    if (!coms_decodePacket) {
        fprintf(stderr, "Failed to load coms_decodePacket function: %s\n", dlerror());
        return -1;
    }

    int ret = coms_decodePacket(&p, p.data, packet_len);

    if (ret == 1) {
        *device_id = p.address;
        *packet_id = p.code;
        *data_len = p.length - PKT_HEADER_LEN;
        *data = malloc(*data_len);
        memcpy(*data, p.data, *data_len);
        *option = p.useOption ? p.option : -1;
        return 0;
    } else {
        fprintf(stderr, "parse_packet(): unknown error code %d\n", ret);
        return -1;
    }
}


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <errno.h>

#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
#include <io.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#define DATA_BYTES_PER_PACKET 64
#define RESPONSE_TIMEOUT 1000 // in microseconds

typedef struct {
    uint8_t length;
    uint8_t address;
    uint16_t code;
    uint16_t crc;
    uint8_t data[DATA_BYTES_PER_PACKET];
    uint8_t transmitData[DATA_BYTES_PER_PACKET];
    uint8_t protocol;
    uint8_t option;
    uint8_t useOption;
    uint16_t receiveRegister;
    uint8_t totalFrames;
} Packet_t;

typedef struct {
    int socket_fd;
    int serial_fd;
    struct sockaddr_in address;
    bool is_serial;
} Connection;

typedef struct {
    uint8_t *incomplete_packets;
    size_t incomplete_size;
} PacketReader;

// Function prototypes
Connection create_serial_connection(const char *serial_port, int baud_rate);
Connection create_socket_connection(const char *ip, int port);
void write_packet(Connection *conn, const uint8_t *packet, size_t length);
ssize_t read_packet(Connection *conn, uint8_t *buffer, size_t buffer_size);
void encode_packet(Packet_t *packet, uint8_t device_id, uint16_t packet_id, const uint8_t *data, size_t data_length);
bool decode_packet(const uint8_t *packet_data, size_t length, Packet_t *decoded_packet);
void sleep_ms(int milliseconds);

// Cross-platform sleep function
void sleep_ms(int milliseconds) {
#ifdef _WIN32
    Sleep(milliseconds);
#else
    usleep(milliseconds * 1000);
#endif
}

// Create a serial connection
Connection create_serial_connection(const char *serial_port, int baud_rate) {
    Connection conn = {0};
    conn.is_serial = true;

#ifdef _WIN32
    HANDLE hSerial = CreateFile(serial_port, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Error opening serial port: %s\n", serial_port);
        exit(EXIT_FAILURE);
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        fprintf(stderr, "Error getting serial port state\n");
        exit(EXIT_FAILURE);
    }

    dcbSerialParams.BaudRate = baud_rate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        fprintf(stderr, "Error setting serial port state\n");
        exit(EXIT_FAILURE);
    }

    conn.serial_fd = (int)hSerial;
#else
    int fd = open(serial_port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Error opening serial port");
        exit(EXIT_FAILURE);
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("Error getting serial port attributes");
        exit(EXIT_FAILURE);
    }

    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN] = 1;                         // read doesn't block
    tty.c_cc[VTIME] = 1;                        // 0.1 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);  // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Error setting serial port attributes");
        exit(EXIT_FAILURE);
    }

    conn.serial_fd = fd;
#endif

    return conn;
}

// Create a socket connection
Connection create_socket_connection(const char *ip, int port) {
    Connection conn = {0};
    conn.is_serial = false;

    conn.socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (conn.socket_fd < 0) {
        perror("Error creating socket");
        exit(EXIT_FAILURE);
    }

    memset(&conn.address, 0, sizeof(conn.address));
    conn.address.sin_family = AF_INET;
    conn.address.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &conn.address.sin_addr) <= 0) {
        perror("Invalid IP address");
        exit(EXIT_FAILURE);
    }

    return conn;
}

// Write a packet to the connection
void write_packet(Connection *conn, const uint8_t *packet, size_t length) {
    if (conn->is_serial) {
#ifdef _WIN32
        DWORD bytes_written;
        WriteFile((HANDLE)conn->serial_fd, packet, length, &bytes_written, NULL);
#else
        write(conn->serial_fd, packet, length);
#endif
    } else {
        sendto(conn->socket_fd, packet, length, 0, (struct sockaddr *)&conn->address, sizeof(conn->address));
    }
}

// Read a packet from the connection
ssize_t read_packet(Connection *conn, uint8_t *buffer, size_t buffer_size) {
    if (conn->is_serial) {
#ifdef _WIN32
        DWORD bytes_read;
        ReadFile((HANDLE)conn->serial_fd, buffer, buffer_size, &bytes_read, NULL);
        return bytes_read;
#else
        return read(conn->serial_fd, buffer, buffer_size);
#endif
    } else {
        socklen_t addr_len = sizeof(conn->address);
        return recvfrom(conn->socket_fd, buffer, buffer_size, 0, (struct sockaddr *)&conn->address, &addr_len);
    }
}

// Encode a packet
void encode_packet(Packet_t *packet, uint8_t device_id, uint16_t packet_id, const uint8_t *data, size_t data_length) {
    packet->length = data_length + 4;
    packet->address = device_id;
    packet->code = packet_id;
    memcpy(packet->data, data, data_length);
    // Add CRC and other fields as needed
}

// Decode a packet
bool decode_packet(const uint8_t *packet_data, size_t length, Packet_t *decoded_packet) {
    if (length < 4) {
        return false;
    }
    decoded_packet->length = packet_data[0];
    decoded_packet->address = packet_data[1];
    decoded_packet->code = (packet_data[2] << 8) | packet_data[3];
    memcpy(decoded_packet->data, &packet_data[4], decoded_packet->length - 4);
    return true;
}