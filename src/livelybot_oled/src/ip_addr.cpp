#include "ip_addr.hpp"

#include <arpa/inet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>

namespace
{
uint32_t g_ip_array[5]{};
}  // namespace

void update_ip_addr()
{
  const char * net_adapter[3] = {LO_NET, ETHERNET, WLAN};
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    return;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  ifr.ifr_addr.sa_family = AF_INET;

  for (int i = 0; i < 3; ++i) {
    std::strncpy(ifr.ifr_name, net_adapter[i], IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFADDR, &ifr) != -1) {
      g_ip_array[i] = reinterpret_cast<struct sockaddr_in *>(&ifr.ifr_addr)->sin_addr.s_addr;
    } else {
      g_ip_array[i] = 0;
    }
  }

  close(fd);
}

uint32_t get_ip_data_u32(uint8_t i)
{
  return g_ip_array[i];
}

uint32_t *get_ip_data_u32_all()
{
  return g_ip_array;
}
