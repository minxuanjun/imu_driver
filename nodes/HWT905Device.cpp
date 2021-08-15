#include <Eigen/Core>
#include <Eigen/Dense>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static int ret;
static int fd;

ros::Publisher pubImu;
#define BAUD 115200 // 115200 for JY61 ,9600 for others

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3>

ypr2R(const Eigen::MatrixBase<Derived> &ypr) {
  typedef typename Derived::Scalar Scalar_t;

  Scalar_t y = ypr(0) / 180.0 * M_PI;
  Scalar_t p = ypr(1) / 180.0 * M_PI;
  Scalar_t r = ypr(2) / 180.0 * M_PI;

  Eigen::Matrix<Scalar_t, 3, 3> Rz;
  Rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;

  Eigen::Matrix<Scalar_t, 3, 3> Ry;
  Ry << cos(p), 0., sin(p), 0., 1., 0., -sin(p), 0., cos(p);

  Eigen::Matrix<Scalar_t, 3, 3> Rx;
  Rx << 1., 0., 0., 0., cos(r), -sin(r), 0., sin(r), cos(r);

  return Rz * Ry * Rx;
}

int uart_open(int fd, const char *pathname) {
  fd = open(pathname, O_RDWR | O_NOCTTY);
  if (-1 == fd) {
    perror("Can't Open Serial Port");
    return (-1);
  } else
    printf("open %s success!\n", pathname);
  if (isatty(STDIN_FILENO) == 0)
    printf("standard input is not a terminal device\n");
  else
    printf("isatty success!\n");
  return fd;
}

int uart_set(int fd, int nSpeed, int nBits, char nEvent, int nStop) {
  struct termios newtio, oldtio;
  if (tcgetattr(fd, &oldtio) != 0) {
    perror("SetupSerial 1");
    printf("tcgetattr( fd,&oldtio) -> %d\n", tcgetattr(fd, &oldtio));
    return -1;
  }
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;
  switch (nBits) {
  case 7:
    newtio.c_cflag |= CS7;
    break;
  case 8:
    newtio.c_cflag |= CS8;
    break;
  }
  switch (nEvent) {
  case 'o':
  case 'O':
    newtio.c_cflag |= PARENB;
    newtio.c_cflag |= PARODD;
    newtio.c_iflag |= (INPCK | ISTRIP);
    break;
  case 'e':
  case 'E':
    newtio.c_iflag |= (INPCK | ISTRIP);
    newtio.c_cflag |= PARENB;
    newtio.c_cflag &= ~PARODD;
    break;
  case 'n':
  case 'N':
    newtio.c_cflag &= ~PARENB;
    break;
  default:
    break;
  }

  /*设置波特率*/

  switch (nSpeed) {
  case 2400:
    cfsetispeed(&newtio, B2400);
    cfsetospeed(&newtio, B2400);
    break;
  case 4800:
    cfsetispeed(&newtio, B4800);
    cfsetospeed(&newtio, B4800);
    break;
  case 9600:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  case 115200:
    cfsetispeed(&newtio, B115200);
    cfsetospeed(&newtio, B115200);
    break;
  case 460800:
    cfsetispeed(&newtio, B460800);
    cfsetospeed(&newtio, B460800);
    break;
  default:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  }
  if (nStop == 1)
    newtio.c_cflag &= ~CSTOPB;
  else if (nStop == 2)
    newtio.c_cflag |= CSTOPB;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;
  tcflush(fd, TCIFLUSH);

  if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
    perror("com set error");
    return -1;
  }
  printf("set done!\n");
  return 0;
}

int uart_close(int fd) {
  assert(fd);
  close(fd);

  return 0;
}
int send_data(int fd, char *send_buffer, int length) {
  length = write(fd, send_buffer, length * sizeof(unsigned char));
  return length;
}

int recv_data(int fd, char *recv_buffer, int length) {
  int has_receive_length = 0;
  while (length - has_receive_length > 0) {

    has_receive_length +=
        read(fd, recv_buffer + has_receive_length, length - has_receive_length);
    usleep(1000);
    //        std::cout << "has_receive_length: " << has_receive_length <<
    //        std::endl;
  }
  return length;
}

float a[3], w[3], Angle[3], h[3];
void ParseData(char *buffer) {
  char chrBuf[100];
  signed short sData[4];

  for (int i = 0; i < 4; i++) {
    memcpy(&chrBuf[0], buffer + 11 * i, 11);

    //        std::cout << "i: " << i << std::endl;
    //        for(int m =0; m< 11; m++)
    //        {
    //            printf("%x  ", chrBuf[m]);
    //        }
    memcpy(&sData[0], &chrBuf[2], 8);
    char cTemp = 0;
    time_t now;
    for (int j = 0; j < 10; j++) {
      cTemp += chrBuf[j];
    }

    if ((chrBuf[0] != 0x55) || ((chrBuf[1] & 0x50) != 0x50) ||
        (cTemp != chrBuf[10])) {
      printf("Error:%x  %x\r\n", chrBuf[0], chrBuf[1]);
      return;
    }
    switch (chrBuf[1]) {
    case 0x51:
      for (int k = 0; k < 3; k++) {
        a[k] = (float)sData[k] / 32768.0 * 16.0 * 9.81;
      }
      time(&now);
      printf("\r\nT:%s a:%6.3f %6.3f %6.3f ", asctime(localtime(&now)), a[0],
             a[1], a[2]);

      break;
    case 0x52:
      for (int k = 0; k < 3; k++) {
        w[k] = (float)sData[k] / 32768.0 * 2000.0/180*M_PI;
      }
      printf("w:%7.3f %7.3f %7.3f ", w[0], w[1], w[2]);
      break;
    case 0x53:
      for (int k = 0; k < 3; k++) {
        Angle[k] = (float)sData[k] / 32768.0 * 180.0;
      }
      printf("A:%7.3f %7.3f %7.3f ", Angle[0], Angle[1], Angle[2]);

      break;

    case 0x54:
      for (int k = 0; k < 3; k++) {
        h[k] = (float)sData[k];
      }
      printf("h:%4.0f %4.0f %4.0f ", h[0], h[1], h[2]);

      break;
    }
  }

  sensor_msgs::Imu imu_msg;

  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = "/imu";
  imu_msg.angular_velocity.x = w[0];
  imu_msg.angular_velocity.y = w[1];
  imu_msg.angular_velocity.z = w[2];

  Eigen::Vector3d ypr;
  ypr << Angle[2], Angle[1], Angle[0];
  Eigen::Matrix R = ypr2R(ypr);
  std::cout << "R:\n " << R << std::endl;
  Eigen::Quaterniond q(R);
  q.normalize();

  imu_msg.orientation.w = q.w();
  imu_msg.orientation.x = q.x();
  imu_msg.orientation.y = q.y();
  imu_msg.orientation.z = q.z();

  imu_msg.linear_acceleration.x = a[0];
  imu_msg.linear_acceleration.y = a[1];
  imu_msg.linear_acceleration.z = a[2];

  pubImu.publish(imu_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "xense_driver");
  ros::NodeHandle n_private("~");

  char r_buf[1024];
  bzero(r_buf, 1024);

  fd = uart_open(fd, "/dev/ttyUSB0"); /*串口号/dev/ttySn,USB口号/dev/ttyUSBn */
  if (fd == -1) {
    fprintf(stderr, "uart_open error\n");
    exit(EXIT_FAILURE);
  }

  if (uart_set(fd, BAUD, 8, 'N', 1) == -1) {
    fprintf(stderr, "uart set failed!\n");
    exit(EXIT_FAILURE);
  }

  pubImu = n_private.advertise<sensor_msgs::Imu>("/imu/data", 3000);

  while (ros::ok()) {
    ret = recv_data(fd, r_buf, 1);

    if (r_buf[0] != 0x55)
      continue;
    ret = recv_data(fd, r_buf + 1, 1);
    if (r_buf[1] != 0x51) {
      continue;
    }

    ret = recv_data(fd, r_buf + 2, 44 - 2);

    //        for(int i =0; i< 44; i++)
    //        {
    //            printf("%x  ", r_buf[i]);
    //        }
    std::cout << std::endl;
    ParseData(r_buf);
    usleep(1000);
  }

  ret = uart_close(fd);
  if (ret == -1) {
    fprintf(stderr, "uart_close error\n");
    exit(EXIT_FAILURE);
  }

  exit(EXIT_SUCCESS);
}