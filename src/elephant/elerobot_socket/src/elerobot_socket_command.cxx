#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <errno.h>  
#include <cstring>
#include "elerobot_socket_command.hxx"

// [Note]: TCP中send和recv函数针对的是字符（无边界）， 而不是字符串（以'\0'作为边界）
// 而sprintf()函数调用的时候会自动在末尾（不是buffer的末尾）加上'\0'，std::strlen()
// 以'\0'划分计算长度，所以可以用这个函数对buf进行有效单元选取。另外，如`sprintf(buf, "%s", "get_angles()");`
// 中的字符串，其本身初始化不会有'\0'在末尾。
using namespace eleRobot;

template<typename T>
int find_element(T str[], T element) {
  int i;
  for (i = 0; i < std::strlen(*str); i++) {
    if (strcmp(element == str[i])) {
      return i;
    }
  }
  return ERROR;
}

base_command::base_command() {
  timeout.tv_sec  = 3; // set timeout time to 3 sec
  timeout.tv_usec = 0; 

  /**
   * @brief Construct a new setsockopt to set receiving and sending timeout
   * 
   * @param param2 SOL_SOCKET:通用套接字选项、IPPROTO_IP:IP层、IPPROTO_TCP:TCP层
   */
  setsockopt(client_sockfd_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(struct timeval));
  setsockopt(client_sockfd_, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(struct timeval));
}

// 2.1 get current angles of robot 
std::vector<double> base_command::get_angles(char *buf) {
  sprintf(buf, "%s", "get_angles()");
  if (send(client_sockfd_, buf, std::strlen(buf), 0) < 0) {
     printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);

     return std::vector<double>();
  }
  if ((len_ = recv(client_sockfd_, buf, BUFSIZ,0)) == -1) {
    perror("recv error"); 

    return std::vector<double>(); 
  }

  std::vector<double> angles;
  char *endPtr = NULL;
  // find the `[` address and move the pointer to the next location
  char *currPtr = buf + find_element<char>(buf, '[') + 1; 
  while (*currPtr != ']') {
    double joint_config = std::strtof(currPtr, &endPtr);
    angles.push_back(joint_config);
    currPtr = endPtr++;
  }

  if (angles.size() != 6) {
    perror("Joint angles number is wrong");

    return std::vector<double>(); 
  }

  if (angles.at(0) == -1.0 && angles.at(1) == -2.0 && angles.at(2) == -3.0 && 
                              angles.at(3) == -4.0 && angles.at(4) == -1.0 && angles.at(5) == -1.0) {
    perror("Error occurred. InvalidAngles() received.");

    return std::vector<double>(); 
  }

  return angles;
}

// 2.2 set the angles of robot 
bool base_command::set_angles(char *buf, double j1, double j2, double j3, double j4, double j5, double j6, double speed) {
  sprintf(buf, "%s%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c", "set_angles(", j1, ',', j2, ',', j3, ',', j4, ',', j5, ',', j6, ',', speed, ')');
  if (send(client_sockfd_, buf, std::strlen(buf), 0) < 0) {
     printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);

     return false;
  }

  if ((len_ = recv(client_sockfd_, buf, BUFSIZ,0)) == -1) {
    perror("recv error"); 

    return false; 
  }
  buf[len_] = '\0';
  printf("%s",buf);

  return true;
}

// 2.13 enable the system 
bool base_command::state_on(char *buf) {
  sprintf(buf, "%s", "state_on()");
  if (send(client_sockfd_, buf, std::strlen(buf), 0) < 0) {
     printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);

     return false;
  }

  if ((len_ = recv(client_sockfd_, buf, BUFSIZ,0)) == -1) {
    perror("recv error"); 

    return false; 
  }
  buf[len_] = '\0';
  printf("%s",buf);

  return true;
}

// 2.14 disable the system 
bool base_command::state_off(char *buf) {
  sprintf(buf, "%s", "state_off()");
  if (send(client_sockfd_, buf, std::strlen(buf), 0) < 0) {
     printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);

     return false;
  }

  if ((len_ = recv(client_sockfd_, buf, BUFSIZ,0)) == -1) {
    perror("recv error"); 

    return false; 
  }
  buf[len_] = '\0';
  printf("%s",buf);

  return true;
}

// 2.15 stop the task 
bool base_command::task_stop(char *buf) {
  sprintf(buf, "%s", "task_stop()");
  if (send(client_sockfd_, buf, std::strlen(buf), 0) < 0) {
     printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);

     return false;
  }

  if ((len_ = recv(client_sockfd_, buf, BUFSIZ,0)) == -1) {
    perror("recv error"); 

    return false; 
  }
  buf[len_] = '\0';
  printf("%s",buf);

  return true;
}

// 2.19 power on the robot 
bool base_command::power_on(char *buf) {
  sprintf(buf, "%s", "power_on()");
  if (send(client_sockfd_, buf, std::strlen(buf), 0) < 0) {
     printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);

     return false;
  }

  if ((len_ = recv(client_sockfd_, buf, BUFSIZ,0)) == -1) {
    perror("recv error"); 

    return false; 
  }
  buf[len_] = '\0';
  printf("%s",buf);

  return true;
}

// 2.20 power off the robot 
bool base_command::power_off(char *buf) {
  sprintf(buf, "%s", "power_off()");
  if (send(client_sockfd_, buf, std::strlen(buf), 0) < 0) {
     printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);

     return false;
  }

  if ((len_ = recv(client_sockfd_, buf, BUFSIZ,0)) == -1) {
    perror("recv error"); 

    return false; 
  }
  buf[len_] = '\0';
  printf("%s",buf);

  return true;
}

// 2.22 check the state of the robot 
bool base_command::state_check(char *buf) {
  sprintf(buf, "%s", "state_check()");
  if (send(client_sockfd_, buf, std::strlen(buf), 0) < 0) {
    printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);

    return false;
  }

  if ((len_ = recv(client_sockfd_, buf, BUFSIZ,0)) == -1) {
    perror("recv error"); 

    return false; 
  }
  buf[len_] = '\0';
  printf("%s",buf);

  return true;
}

// 2.23 check if the robot is running 
std::pair<bool, int> base_command::check_running(char *buf) {
  sprintf(buf, "%s", "check_running()");
  if (send(client_sockfd_, buf, std::strlen(buf), 0) < 0) {
    printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);

    return std::make_pair(false, ERROR);
  }

  if ((len_ = recv(client_sockfd_, buf, BUFSIZ,0)) == -1) {
    perror("recv error"); 

    return std::make_pair(false, ERROR); 
  }
  
  char *currPtr = buf + find_element<char>(buf, ':') + 1;
  int res = atoi(currPtr);
  if (res == 1) {
    printf("Robot is running\n");

    return std::make_pair(true, res);
  } 
  else if (res == 0) {
    printf("Robot is not running\n");

    return std::make_pair(false, res);
  } 
  else {
    buf[len_] = '\0';
    printf("%s",buf);

    return std::make_pair(false, ERROR); 
  }

}

// 2.26 get the robot error 
bool base_command::read_next_error(char *buf) {
  sprintf(buf, "%s", "read_next_error()");
  if (send(client_sockfd_, buf, std::strlen(buf), 0) < 0) {
    printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);

    return false;
  }

  if ((len_ = recv(client_sockfd_, buf, BUFSIZ,0)) == -1) {
    perror("recv error"); 

    return false; 
  }
  buf[len_] = '\0';
  printf("%s",buf);

  return true;
}

// 2.32 wait for command done
bool base_command::read_next_error(char *buf) {
  sprintf(buf, "%s", "wait_command_done()");
  if (send(client_sockfd_, buf, std::strlen(buf), 0) < 0) {
    printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);

    return false;
  }

  if ((len_ = recv(client_sockfd_, buf, BUFSIZ,0)) == -1) {
    perror("recv error"); 

    return false; 
  }
  char *currPtr = buf + find_element<char>(buf, ':') + 1;
  int res = atoi(currPtr);
  if (res == 0) {
    printf("previous command done");
    
    return true;
  } 
  else {
    buf[len_] = '\0';
    printf("%s",buf);
    return false;
  }

  return true;
}

// 2.32 pause the program
bool base_command::pause_program(char *buf) {
  sprintf(buf, "%s", "pause_program()");
  if (send(client_sockfd_, buf, std::strlen(buf), 0) < 0) {
    printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);

    return false;
  }

  if ((len_ = recv(client_sockfd_, buf, BUFSIZ,0)) == -1) {
    perror("recv error"); 

    return false; 
  }
  buf[len_] = '\0';
  printf("%s",buf);

  return true;
}

// 2.33 resume the program 
bool base_command::resume_program(char *buf) {
  sprintf(buf, "%s", "resume_program()");
  if (send(client_sockfd_, buf, std::strlen(buf), 0) < 0) {
    printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);

    return false;
  }

  if ((len_ = recv(client_sockfd_, buf, BUFSIZ,0)) == -1) {
    perror("recv error"); 

    return false; 
  }
  buf[len_] = '\0';
  printf("%s",buf);

  return true;
}