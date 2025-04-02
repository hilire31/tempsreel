/*
 * File:          my_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/utils/ansi_codes.h>

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h> /* definition of close */


#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <arpa/inet.h>  /* definition of inet_ntoa */
#include <netdb.h>      /* definition of gethostbyname */
#include <netinet/in.h> /* definition of struct sockaddr_in */


#define SOCKET_PORT 10020

static int fd;
static fd_set rfds;

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64


WbDeviceTag ds0, ds1, left_motor, right_motor, left_sensor, right_sensor, inertial_unit;

double left_speed = 0, right_speed = 0;
double max_speed;
double target_ang_position = 0.0;

int mode = 0;
  
char buffer[256];
struct timeval tv = {0, 0};
int number, ret, n;


static int accept_client(int server_fd) {
  int cfd;
  struct sockaddr_in client;
  socklen_t asize;
  const struct hostent *client_info;

  asize = sizeof(struct sockaddr_in);

  cfd = accept(server_fd, (struct sockaddr *)&client, &asize);
  if (cfd == -1) {
    printf("cannot accept client\n");
    return -1;
  }
  client_info = gethostbyname((char *)inet_ntoa(client.sin_addr));
  printf("Accepted connection from: %s \n", client_info->h_name);

  return cfd;
}

static int create_socket_server(int port) {
  int sfd, rc;
  struct sockaddr_in address;

  /* create the socket */
  sfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sfd == -1) {
    printf("cannot create socket\n");
    return -1;
  }

  /* fill in socket address */
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons((unsigned short)port);
  address.sin_addr.s_addr = INADDR_ANY;

  /* bind to port */
  printf("Binding to port %d...\n", port);
  rc = bind(sfd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    printf("cannot bind port %d\n", port);
    close(sfd);
    return -1;
  }

  /* listen for connections */
  printf("Waiting for a connection on port %d...\n", port);
  if (listen(sfd, 1) == -1) {
    printf("cannot listen for connections\n");
    close(sfd);
    return -1;
  }
  
  return accept_client(sfd);
}


void get_orders(void) {
    int number;

    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    
    /*
     * Watch TCPIP file descriptor to see when it has input.
     * No wait - polling as fast as possible
     */
    number = select(fd + 1, &rfds, NULL, NULL, &tv);
    /* If no new data, jump to the next time step */
    if(number==0) return;
    
    n = recv(fd, buffer, 256, 0);
    if (n < 0) {
      printf("error reading from socket\n");
      return;
    }
    else if (n>0) { 
      buffer[n] = '\0';
      printf("Received %d bytes: %s\n", n, buffer);
    }
    
    if (buffer[0] == 'M') { /* set the speed of the motors */
      int new_left_speed, new_right_speed;
      sscanf(buffer, "M,%d,%d", &new_left_speed, &new_right_speed);
      left_speed  =  new_left_speed  * max_speed * 0.01;
      right_speed =  new_right_speed * max_speed * 0.01;
      wb_motor_set_velocity(left_motor,  left_speed);
      wb_motor_set_velocity(right_motor, right_speed);
      printf("Setting motors %.2lf %.2lf\n",left_speed, right_speed);
    }
    else if (buffer[0] == 'S') { /* Read distance sensors */
     double ds0_value = wb_distance_sensor_get_value(ds0);
     double ds1_value = wb_distance_sensor_get_value(ds1);
     char reply[256];
     sprintf(reply, "S,%.3f,%.3f\n", ds0_value, ds1_value);
     send(fd,reply,strlen(reply),0); 
     printf("Distance sensor 0 value: %.3f\n",ds0_value);
     printf("Distance sensor 1 value: %.3f\n",ds1_value);
    } 
    else if (buffer[0] == 'W') { /* Read wheel sensors odo */
      double left_pos  = wb_position_sensor_get_value(left_sensor);
      double right_pos = wb_position_sensor_get_value(right_sensor);
      char reply[256];
      sprintf(reply, "W,%.3f,%.3f\n", left_sensor, right_sensor);
      printf("Wheel Positions: Left=%.2f, Right=%.2f\n", left_pos, right_pos); 
    }
    else if (buffer[0] == 'B') { /* Read battery level */
      const double battery_value = wb_robot_battery_sensor_get_value();
      char reply[256];
       sprintf(reply, "B,%.3f\n", battery_value);
       send(fd,reply,strlen(reply), 0); 
       printf("Battery level: %.3f\n", battery_value);
    } 
    else if (buffer[0] == 'T') { /* Turn robot */
      sscanf(buffer, "T,%lf", &target_ang_position);
      mode = turn_robot(target_ang_position);
      if(mode==0) {
        send(fd,"T",strlen("T"), 0); 
      }
    }
}

int turn_robot(double target_ang_position) {
    const double * inertial_unit_readings = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
    double delta = inertial_unit_readings[2] - target_ang_position;
    
    if (fabs(delta) < 0.01) {
      wb_motor_set_velocity(left_motor,  0);
      wb_motor_set_velocity(right_motor, 0);
      return 0;
    }
   
   int dir = (delta > 0) ? 1 : ((delta < 0) ? -1 : 0);
   wb_motor_set_velocity(left_motor,   dir * 0.5);
   wb_motor_set_velocity(right_motor, -dir * 0.5); 
   return 1;
}


int main(int argc, char **argv) {

  wb_robot_init();

  ds0 = wb_robot_get_device("ds0");
  ds1 = wb_robot_get_device("ds1");
  left_sensor = wb_robot_get_device("left wheel sensor");
  right_sensor = wb_robot_get_device("right wheel sensor");
  inertial_unit = wb_robot_get_device("inertial unit");
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  max_speed = wb_motor_get_max_velocity(left_motor);

  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);
  wb_robot_battery_sensor_enable(TIME_STEP);
  wb_position_sensor_enable(left_sensor, TIME_STEP);
  wb_position_sensor_enable(right_sensor, TIME_STEP);
  wb_inertial_unit_enable(inertial_unit, TIME_STEP);
  
  
  fd = create_socket_server(SOCKET_PORT);
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);

  while (wb_robot_step(TIME_STEP) != -1) { 
    if(mode) {
      mode = turn_robot(target_ang_position);
      if(mode==0) 
        send(fd,"T",strlen("T"),0);  
    }
    else {
      get_orders();
    } 
  }
  
  
  close(fd);
  wb_robot_cleanup();

  return 0;
}
