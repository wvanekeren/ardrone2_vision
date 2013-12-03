


#include <stdint.h>
#include "udp/socket.h"

void send_rtp_frame(struct UdpSocket *sock, uint8_t * Jpeg, uint32_t JpegLen, int w, int h);
void test_rtp_frame(struct UdpSocket *sock);
