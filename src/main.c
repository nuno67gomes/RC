// // Main file of the serial port project.
// // DO NOT CHANGE THIS FILE

// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>

// #include "application_layer.h"

// #define N_TRIES 3
// #define TIMEOUT 4

// // Arguments:
// //   $1: /dev/ttySxx
// //   $2: baud rate
// //   $3: tx | rx
// //   $4: filename
// int main(int argc, char *argv[])
// {
//     if (argc < 5)
//     {
//         printf("Usage: %s /dev/ttySxx baudrate tx|rx filename\n", argv[0]);
//         exit(1);
//     }

//     const char *serialPort = argv[1];
//     const int baudrate = atoi(argv[2]);
//     const char *role = argv[3];
//     const char *filename = argv[4];

//     // Validate baud rate
//     switch (baudrate)
//     {
//     case 1200:
//     case 1800:
//     case 2400:
//     case 4800:
//     case 9600:
//     case 19200:
//     case 38400:
//     case 57600:
//     case 115200:
//         break;
//     default:
//         printf("Unsupported baud rate (must be one of 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200)\n");
//         exit(2);
//     }

//     // Validate role
//     if (strcmp("tx", role) != 0 && strcmp("rx", role) != 0)
//     {
//         printf("ERROR: Role must be \"tx\" or \"rx\"\n");
//         exit(3);
//     }

//     printf("Starting link-layer protocol application\n"
//            "  - Serial port: %s\n"
//            "  - Role: %s\n"
//            "  - Baudrate: %d\n"
//            "  - Number of tries: %d\n"
//            "  - Timeout: %d\n"
//            "  - Filename: %s\n",
//            serialPort,
//            role,
//            baudrate,
//            N_TRIES,
//            TIMEOUT,
//            filename);

//     applicationLayer(serialPort, role, baudrate, N_TRIES, TIMEOUT, filename);

//     return 0;
// }


#include "link_layer.h"
#include <stdio.h>
#include <string.h>

int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("Usage: %s <serial_port> <tx|rx>\n", argv[0]);
        return 1;
    }

    LinkLayer ll;

    // Copy serial port path
    snprintf(ll.serialPort, sizeof(ll.serialPort), "%s", argv[1]);

    // Default configuration
    ll.baudRate = 9600;
    ll.nRetransmissions = 3;
    ll.timeout = 3;
    ll.role = (strcmp(argv[2], "tx") == 0) ? LlTx : LlRx;

    printf("Starting llopen() as %s on %s\n",
           ll.role == LlTx ? "Transmitter" : "Receiver",
           ll.serialPort);

    int r = llopen(ll);

    if (r == 0)
        printf("llopen() success!\n");
    else
        printf("llopen() failed!\n");

    return 0;
}
