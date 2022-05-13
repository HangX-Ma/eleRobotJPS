/**
 * @file elerobot_basesocket.hpp
 * @brief basic socket communication
 * @version 0.1
 * @date 2022-05-13
 */
#ifndef ELEROBOT_BASESOCKET_HPP
#define ELEROBOT_BASESOCKET_HPP

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <netdb.h>
#include <string>
#include <functional>
#include <cerrno>

#define FDR_UNUSED(expr){ (void)(expr); } 
#define FDR_ON_ERROR std::function<void(int, std::string)> onError = [](int errorCode, std::string errorMessage){FDR_UNUSED(errorCode); FDR_UNUSED(errorMessage);}
// https://stackoverflow.com/questions/20353210/usage-and-syntax-of-stdfunction
// [FDR_UNUSED]: suppress compiler warnings of unused variables/arguments to functions.
class BaseSocket
{
  public:
    enum SocketType {
        TCP = SOCK_STREAM,
        UDP = SOCK_DGRAM
    };
    const uint16_t BUFFER_SIZE = 0xFFFF;
    sockaddr_in address;
    bool isClosed = false;

  protected:
    //! socket FD
    int sock = 0;
    static std::string ipToString(sockaddr_in addr) {
        char ip[INET_ADDRSTRLEN];
        /**
         * @brief Convert the numeric format to the Dot-decimal notation IP address format
         * 
         * @param AF_INET IPv4
         * @param AF_INET6 IPv6
         */
        inet_ntop(AF_INET, &(addr.sin_addr), ip, INET_ADDRSTRLEN);

        return std::string(ip);
    }

    BaseSocket(FDR_ON_ERROR, SocketType sockType = TCP, int socketId = -1)
    {
        if (socketId < 0) {
            if ((this->sock = socket(AF_INET, sockType, 0)) < 0) {
                onError(errno, "Socket creating error.");
            }
        }
        else {
            this->sock = socketId;
        }
    }
  // Methods
  public:
      virtual void Close() {
          if(isClosed) return;

          isClosed = true;
          close(this->sock);
      }

      std::string remoteAddress() {return ipToString(this->address);}
      int remotePort() {return ntohs(this->address.sin_port);}
      int fileDescriptor() const { return this->sock; }
};

#endif