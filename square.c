/*
 * An example SMR program.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

#define NUM_LINE_SENSORS 8

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct
{
  double x, y, z, omega, phi, kappa, code, id, crc;
} gmk;
double visionpar[10];
double laserpar[10];
volatile int running = 1;
int robot_port;

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv, camsrv;

symTableElement *
getinputref(const char *sym_name, symTableElement *tab)
{
  int i;
  for (i = 0; i < getSymbolTableSize('r'); i++)
    if (strcmp(tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}

symTableElement *
getoutputref(const char *sym_name, symTableElement *tab)
{
  int i;
  for (i = 0; i < getSymbolTableSize('w'); i++)
    if (strcmp(tab[i].name, sym_name) == 0)
      return &tab[i];
  return 0;
}
/*****************************************
 * odometry
 */

// #define ED1 0.9919
// #define ED2 0.9963
// #define WHEEL_SEPARATION 0.2369 /* m */
#define ED1 1
#define ED2 1
#define WHEEL_SEPARATION 0.26 /* m */
#define WHEEL_DIAMETER_L 0.06522 * (1 + (1 - ED1) / 2) * (1 + (1 - ED2) / 2)/* m */
#define WHEEL_DIAMETER_R 0.06522 * (1 - (1 - ED1) / 2) * (1 - (1 - ED2) / 2)
#define DELTA_M_L (M_PI * WHEEL_DIAMETER_L / 2000)
#define DELTA_M_R (M_PI * WHEEL_DIAMETER_R / 2000)
#define DEFAULT_ROBOTPORT 24902
#define EPSILON 0.01
#define K 2.0
#define KP 10.0
#define KI 0.0
#define KD 2.0
#define ACCELERATION 1.0

// #define BLACK_VALUE 59
// #define WHITE_VALUE 95
#define BLACK_VALUE 0
#define WHITE_VALUE 255

#define BLACK_LINE_FOUND_VALUE 0.1 /* If all the line sensors are below this value, then the robot must have crossed a black line */
#define WHITE_LINE_FOUND_VALUE 0.9 /* If all the line sensors are above this value, then the robot must have crossed a white line */


typedef struct
{                          // input signals
  int left_enc, right_enc; // encoderticks
  // parameters
  double w;      // wheel separation
  double cr, cl; // meters per encodertick
  // output signals
  double right_pos, left_pos;
  double pos, theta, x, y;
  // internal variables
  int left_enc_old, right_enc_old;
  double error_cur, error_old, error_all;
} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);

/********************************************
 * Motion control
 */

typedef struct
{ // input
  int cmd;
  int curcmd, color;
  double speedcmd;
  double dist;
  double start_theta;
  double angle;
  double left_pos, right_pos;
  // parameters
  double w;
  // output
  double motorspeed_l, motorspeed_r;
  double motorspeed_l_old, motorspeed_r_old;
  double motoraccelerate_l, motoraccelerate_r;
  int finished;
  // internal variables
  double startpos;
} motiontype;

enum
{
  mot_stop = 1,
  mot_move,
  mot_turn,
  mot_follow_black_l,
  mot_follow_black_r,
};

typedef struct
{
  int left, right, length, find_l, find_r, crossline, left_white, right_white, find_l_white, find_r_white, crossline_white;
  double left_pos, right_pos, left_pos_white, right_pos_white;
  double line_raw[8], line_calibrate[8];
} linesensortype;

typedef struct
{
  int left, right, length, find_l, find_r, crossline;
  double left_pos, right_pos, middle_pos;
} irsensortype;

typedef struct
{
  int left, right, length, find_l, find_r, crossline;
  double left_pos, right_pos, middle_pos;
} lasersensortype;

void update_motcon(motiontype *p, odotype *q, linesensortype *line);
int fwd(double dist, double speed, int time);
int turn(double angle, double speed, int time);
int follow_black_l(double speed, double dist, int color, int time);
int follow_black_r(double speed, double dist, int color, int time);

void segfaulthandler(int sig)
{
  //    perror(NULL);
  printf("Seg-error\n");
  exit(1);
}

void brokenpipehandler(int sig)
{
  printf("mrc: broken pipe \n");
  // savelog("log");
  exit(1);
}

void ctrlchandler(int sig)
{
  printf("mrc: ctrl-c \n");
  running = 0;
}

// typedef struct
// {
//   int state, oldstate;
//   int time;
// } smtype;

typedef struct
{
  int state, oldstate;
  int states_set[1000];
  double dist[1000], angle[1000], speed[1000], color[1000];
  int state_index;
  int time;
} smtype;

void sm_update(smtype *p);
int crossline(int i, double color, double *data);
void update_linesensor(symTableElement *linesensor, linesensortype *line, double w);
void calPos_linesensor(linesensortype *line, double w);
void square(smtype *p, double dist, double direction, double speed);
void mission_fwd(smtype *p, int i, double dist, double speed);
void mission_turn(smtype *p, int i, double angle, double speed);
void mission_follow_black_l_line(smtype *p, int i, double speed, double dist, int color);
void mission_follow_black_r_line(smtype *p, int i, double speed, double dist, int color);
void mission_1(smtype *p);

// SMR input/output data

symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl;

odotype odo;
smtype mission;
motiontype mot;
linesensortype line;

enum
{
  ms_init,
  ms_fwd,
  ms_turn,
  ms_follow_black_l,
  ms_follow_black_r,
  ms_end
};

int logcount = 0;

int main(int argc, char **argv)
{
  int n = 0, arg, time = 0, opt, calibration;
  double dist = 0, angle = 0;
  // install sighandlers
  if (1)
  {
    if (signal(SIGSEGV, segfaulthandler) == SIG_ERR)
    {
      perror("signal");
      exit(1);
    }
  }
  if (signal(SIGPIPE, brokenpipehandler) == SIG_ERR)
  {
    perror("signal");
    exit(1);
  }
  if (signal(SIGINT, ctrlchandler) == SIG_ERR)
  {
    perror("signal");
    exit(1);
  }
  robot_port = DEFAULT_ROBOTPORT;
  while (EOF != (opt = getopt(argc, argv, "ct:v:l:s:h:u")))
  {
    switch (opt)
    {
    case 'c':
      calibration = 1;
      break;

    case 's':
      if (optarg)
      {
        int port;
        port = atoi(optarg);
        if (port != 0)
          robot_port = port;
      }
      else
        exit(1);
      break;

    default:;
    }
  }

  /* Establish connection to robot sensors and actuators.
   */
  if (rhdConnect('w', "localhost", robot_port) != 'w')
  {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }

  printf("connected to robot \n");
  if ((inputtable = getSymbolTable('r')) == NULL)
  {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }
  if ((outputtable = getSymbolTable('w')) == NULL)
  {
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }
  // connect to robot I/O variables
  lenc = getinputref("encl", inputtable);
  renc = getinputref("encr", inputtable);
  linesensor = getinputref("linesensor", inputtable);
  irsensor = getinputref("irsensor", inputtable);

  speedl = getoutputref("speedl", outputtable);
  speedr = getoutputref("speedr", outputtable);
  resetmotorr = getoutputref("resetmotorr", outputtable);
  resetmotorl = getoutputref("resetmotorl", outputtable);

  // **************************************************
  //  Camera server code initialization
  //

  /* Create endpoint */
  lmssrv.port = 24919;
  strcpy(lmssrv.host, "127.0.0.1");
  strcpy(lmssrv.name, "laserserver");
  lmssrv.status = 1;
  camsrv.port = 24920;
  strcpy(camsrv.host, "127.0.0.1");
  camsrv.config = 1;
  strcpy(camsrv.name, "cameraserver");
  camsrv.status = 1;

  if (camsrv.config)
  {
    int errno = 0;
    camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (camsrv.sockfd < 0)
    {
      perror(strerror(errno));
      fprintf(stderr, " Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&camsrv);

    xmldata = xml_in_init(4096, 32);
    printf(" camera server xml initialized \n");
  }

  // **************************************************
  //  LMS server code initialization
  //

  /* Create endpoint */
  lmssrv.config = 1;
  if (lmssrv.config)
  {
    char buf[256];
    int errno = 0, len;
    lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (lmssrv.sockfd < 0)
    {
      perror(strerror(errno));
      fprintf(stderr, " Can not make  socket\n");
      exit(errno);
    }

    serverconnect(&lmssrv);
    if (lmssrv.connected)
    {
      xmllaser = xml_in_init(4096, 32);
      printf(" laserserver xml initialized \n");
      //  len=sprintf(buf,"scanpush cmd='zoneobst'\n");
      len = sprintf(buf, "push  t=0.2 cmd='mrcobst width=0.4'\n");
      send(lmssrv.sockfd, buf, len, 0);
    }
  }

  /* Read sensors and zero our position.
   */
  rhdSync();

  odo.w = WHEEL_SEPARATION;
  odo.cr = DELTA_M_R;
  odo.cl = DELTA_M_L;
  odo.left_enc = lenc->data[0];
  odo.right_enc = renc->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w = odo.w;
  running = 1;
  // square(&mission, 1.0, 1.0, 0.3);
  mission_1(&mission);
  while (running)
  {
    if (lmssrv.config && lmssrv.status && lmssrv.connected)
    {
      while ((xml_in_fd(xmllaser, lmssrv.sockfd) > 0))
        xml_proca(xmllaser);
    }

    if (camsrv.config && camsrv.status && camsrv.connected)
    {
      while ((xml_in_fd(xmldata, camsrv.sockfd) > 0))
        xml_proc(xmldata);
    }

    rhdSync();
    odo.left_enc = lenc->data[0];
    odo.right_enc = renc->data[0];
    update_odo(&odo);

    /****************************************
    / mission statemachine
    */
    sm_update(&mission);

    switch (mission.state)
    {
    case ms_init:
      // n = 4;
      // dist = 1;
      // angle = -90.0 / 180 * M_PI;
      // mission.state = ms_fwd;
      mission.state_index++;
      mission.state = mission.states_set[mission.state_index];
      // mission.state = ms_follow_black_l;
      break;

    case ms_fwd:

      if (fwd(mission.dist[mission.state_index], mission.speed[mission.state_index], mission.time)){
        // mission.state = ms_turn;
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_turn:
      if (turn(mission.angle[mission.state_index], mission.speed[mission.state_index], mission.time)){
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      // {
      //   n = n - 1;
      //   if (n == 0)
      //     mission.state = ms_end;
      //   else
      //     mission.state = ms_fwd;
      // }
      break;

    case ms_follow_black_l:
      if (follow_black_l(mission.speed[mission.state_index], mission.dist[mission.state_index], mission.color[mission.state_index], mission.time))
      {
        // mission.state = ms_end;
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_follow_black_r:
      if (follow_black_r(mission.speed[mission.state_index], mission.dist[mission.state_index], mission.color[mission.state_index], mission.time))
      {
        // mission.state = ms_end;
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_end:
      mot.cmd = mot_stop;
      running = 0;
      break;
    }
    /*  end of mission  */

    mot.left_pos = odo.left_pos;
    mot.right_pos = odo.right_pos;
    update_motcon(&mot, &odo, &line);
    speedl->data[0] = 100 * mot.motorspeed_l;
    speedl->updated = 1;
    speedr->data[0] = 100 * mot.motorspeed_r;
    speedr->updated = 1;

    update_linesensor(linesensor, &line, odo.w);
    if(logcount == 0){
      FILE *fp;
      fp = fopen("log.dat", "w");
      fprintf(fp, "%d %f %f %f %f %f \n", logcount, mot.motorspeed_l, mot.motorspeed_r, odo.x, odo.y, odo.theta);
      fclose(fp);
    }
    else{
      FILE *fp;
      fp = fopen("log.dat", "a");
      fprintf(fp, "%d %f %f %f %f %f \n", logcount, mot.motorspeed_l, mot.motorspeed_r, odo.x, odo.y, odo.theta);
      fclose(fp);
    }

    if(logcount == 0){
      FILE *fp;
      fp = fopen("linesensorlog.dat", "w");
      for (int i = 0; i < linesensor->length; i++)
      {
        fprintf(fp, "%d ", linesensor->data[i]);
      }
      for (int i = 0; i < linesensor->length; i++)
      {
        fprintf(fp, "%f ", line.line_calibrate[i]);
      }
      fprintf(fp, "%d ", line.left);
      fprintf(fp, "%d ", line.right);
      fprintf(fp, "%f ", line.left_pos);
      fprintf(fp, "%f ", line.right_pos);
      fprintf(fp, "%d ", line.find_l);
      fprintf(fp, "%d ", line.left_white);
      fprintf(fp, "%d ", line.right_white);
      fprintf(fp, "%f ", line.left_pos_white);
      fprintf(fp, "%f ", line.right_pos_white);
      fprintf(fp, "%d ", line.find_l_white);
      fprintf(fp, "\n");
      fclose(fp);
    }
    else{
      FILE *fp;
      fp = fopen("linesensorlog.dat", "a");
      for (int i = 0; i < linesensor->length; i++)
      {
        fprintf(fp, "%d ", linesensor->data[i]);
      }
      for (int i = 0; i < linesensor->length; i++)
      {
        fprintf(fp, "%f ", line.line_calibrate[i]);
      }
      fprintf(fp, "%d ", line.left);
      fprintf(fp, "%d ", line.right);
      fprintf(fp, "%f ", line.left_pos);
      fprintf(fp, "%f ", line.right_pos);
      fprintf(fp, "%d ", line.find_l);
      fprintf(fp, "%d ", line.left_white);
      fprintf(fp, "%d ", line.right_white);
      fprintf(fp, "%f ", line.left_pos_white);
      fprintf(fp, "%f ", line.right_pos_white);
      fprintf(fp, "%d ", line.find_l_white);
      fprintf(fp, "\n");
      fclose(fp);
    }
    
    if(logcount == 0){
      FILE *fp;
      fp = fopen("irsensorlog.dat", "w");
      for (int i = 0; i < irsensor->length; i++)
      {
        fprintf(fp, "%d ", irsensor->data[i]);
      }
      fprintf(fp, "\n");
      fclose(fp);
    }
    else{
      FILE *fp;
      fp = fopen("irsensorlog.dat", "a");
      for (int i = 0; i < irsensor->length; i++)
      {
        fprintf(fp, "%d ", irsensor->data[i]);
      }
      fprintf(fp, "\n");
      fclose(fp);
    }

    logcount++;

    /*insert data collection here*/

    if (time % 100 == 0)
      //    printf(" laser %f \n",laserpar[3]);
      time++;
    /* stop if keyboard is activated
     *
     */
    ioctl(0, FIONREAD, &arg);
    if (arg != 0)
      running = 0;

  } /* end of main control loop */
  printf("logcount: %d\n, linesensor: %d\n", logcount, linesensor->length);
  printf("linesensor: %d\n", linesensor->length);
  speedl->data[0] = 0;
  speedl->updated = 1;
  speedr->data[0] = 0;
  speedr->updated = 1;

  rhdSync();
  rhdDisconnect();

  exit(0);
}

/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */

void reset_odo(odotype *p)
{
  p->right_pos = p->left_pos = 0.0;
  p->pos = 0.0;
  p->theta = 0.0;
  p->x = p->y = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
}

void update_odo(odotype *p)
{
  int delta;
  delta = p->right_enc - p->right_enc_old;
  if (delta > 0x8000)
    delta -= 0x10000;
  else if (delta < -0x8000)
    delta += 0x10000;
  p->right_enc_old = p->right_enc;
  p->right_pos += delta * p->cr;
  double right_delta = delta * p->cr;

  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000)
    delta -= 0x10000;
  else if (delta < -0x8000)
    delta += 0x10000;
  p->left_enc_old = p->left_enc;
  p->left_pos += delta * p->cl;
  double left_delta = delta * p->cr;

  double delta_U = (right_delta + left_delta) / 2;
  double delta_theta = (right_delta - left_delta) / p->w;
  p->pos += delta_U;
  p->theta += delta_theta;
  p->x += delta_U * cos(p->theta);
  p->y += delta_U * sin(p->theta);
}

// exercise 3.5
double accelerate_speed(double old_speed, double desired_speed, double dist, double accelerate)
{
  double vmax = old_speed + accelerate * 10;
  if (vmax > desired_speed)
  {
    vmax = desired_speed;
  }
  if (vmax > sqrt(2.0 * accelerate * dist))
  {
    vmax = sqrt(2.0 * accelerate * dist);
  }
  return vmax;
}

double angular_control(double desired_theta, double current_theta, double k, double w)
{
  double turn_angle_speed = k * (desired_theta - current_theta);
  return turn_angle_speed * w * 0.5 + EPSILON;
}

// exercise 3.4
// double accelerate_speed(double old_speed, double desired_speed, double dist, double accelerate){
//   double vmax = old_speed + accelerate * 10;
//   if(vmax > desired_speed){
//     vmax = desired_speed;
//   }
//   return vmax;
// }

void update_motcon(motiontype *p, odotype *q, linesensortype *line)
{

  if (p->cmd != 0)
  {

    p->finished = 0;
    switch (p->cmd)
    {
    case mot_stop:
      p->curcmd = mot_stop;
      break;
    case mot_move:
      p->start_theta = q->theta;
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_move;
      break;

    case mot_follow_black_l:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_follow_black_l;
      break;

    case mot_follow_black_r:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = mot_follow_black_r;
      break;

    case mot_turn:
      p->start_theta = q->theta;
      if (p->angle > 0)
        p->startpos = p->right_pos;
      else
        p->startpos = p->left_pos;
      p->curcmd = mot_turn;
      break;
    }

    p->cmd = 0;
  }

  double desire_theta = p->angle + p->start_theta;
  double delta_theta = desire_theta - q->theta;
  double direction = fabs(p->angle) / p->angle;
  switch (p->curcmd)
  {
  case mot_stop:
    p->motorspeed_l = 0;
    p->motorspeed_r = 0;
    break;
  case mot_move:

    if (fabs((p->right_pos + p->left_pos) / 2 - p->startpos) > fabs(p->dist))
    {
      p->finished = 1;
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
    }
    else
    {
      // p->motorspeed_l = p->speedcmd;
      // p->motorspeed_r = p->speedcmd;
      double direction = fabs(p->dist) / p->dist;
      double dist = fabs(p->dist - direction * ((p->right_pos + p->left_pos) / 2 - p->startpos));
      p->motorspeed_l_old = p->motorspeed_l;
      p->motorspeed_r_old = p->motorspeed_r;
      // p->motorspeed_l = direction * accelerate_speed(fabs(p->motorspeed_l_old), p->speedcmd - direction * angular_control(p->start_theta, q->theta, K, p->w), dist, 0.5);
      // p->motorspeed_r = direction * accelerate_speed(fabs(p->motorspeed_r_old), p->speedcmd + direction * angular_control(p->start_theta, q->theta, K, p->w), dist, 0.5);
      p->motorspeed_l = direction * accelerate_speed(fabs(p->motorspeed_l_old), p->speedcmd, dist, ACCELERATION);
      p->motorspeed_r = direction * accelerate_speed(fabs(p->motorspeed_r_old), p->speedcmd, dist, ACCELERATION);
    }
    break;

  case mot_turn:
    if (direction * delta_theta >= 0)
    {
      // p->motorspeed_l=-direction * p->speedcmd;
      // p->motorspeed_r=direction * p->speedcmd;
      // exercise 3.6
      double dist = fabs(0.5 * delta_theta * p->w);
      p->motorspeed_l_old = p->motorspeed_l;
      p->motorspeed_r_old = p->motorspeed_r;
      p->motorspeed_r = direction * accelerate_speed(fabs(p->motorspeed_r_old), p->speedcmd, dist, ACCELERATION);
      p->motorspeed_l = -direction * accelerate_speed(fabs(p->motorspeed_r_old), p->speedcmd, dist, ACCELERATION);
      // p->motorspeed_r = direction * angular_control(desire_theta, q->theta, K, p->w);
      // p->motorspeed_l = -direction * angular_control(desire_theta, q->theta, K, p->w);
    }
    else
    {
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
      p->finished = 1;
    }
    break;

  case mot_follow_black_l:
    if ((line->find_l || line->find_l_white) && (!(line->crossline)) && fabs((p->right_pos + p->left_pos) / 2 - p->startpos) < fabs(p->dist))
    {
      double error;
      if(line->find_l){
        error = line->left_pos * fabs(p->speedcmd);
      }
      else{
        error = line->left_pos_white * fabs(p->speedcmd);
      }
      double error_black_D = error - q->error_old;
      double direction = fabs(p->speedcmd) / p->speedcmd;
      q->error_all += error;
      q->error_old = error;
      p->motorspeed_l_old = p->motorspeed_l;
      p->motorspeed_r_old = p->motorspeed_r;
      p->motorspeed_r = direction * (fabs(p->speedcmd) + KP * error + KI * q->error_all + KD * error_black_D);
      p->motorspeed_l = direction * (fabs(p->speedcmd) - KP * error - KI * q->error_all - KD * error_black_D);
    }
    else
    {
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
      p->finished = 1;
      q->error_all = 0;
      q->error_old = 0;
    }
    break;

  case mot_follow_black_r:
    if ((line->find_r || line->find_r_white) && (!(line->crossline)) && fabs((p->right_pos + p->left_pos) / 2 - p->startpos) < fabs(p->dist))
    {
      double error;
      if(line->find_r){
        error = line->right_pos * fabs(p->speedcmd);
      }
      else{
        error = line->right_pos_white * fabs(p->speedcmd);
      }
      double error_black_D = error - q->error_old;
      double direction = fabs(p->speedcmd) / p->speedcmd;
      q->error_all += error;
      q->error_old = error;
      p->motorspeed_l_old = p->motorspeed_l;
      p->motorspeed_r_old = p->motorspeed_r;
      p->motorspeed_r = direction * (fabs(p->speedcmd) + KP * error + KI * q->error_all + KD * error_black_D);
      p->motorspeed_l = direction * (fabs(p->speedcmd) - KP * error - KI * q->error_all - KD * error_black_D);
    }
    else
    {
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
      p->finished = 1;
      q->error_all = 0;
      q->error_old = 0;
    }
    break;
  }
}

int fwd(double dist, double speed, int time)
{
  if (time == 0)
  {
    mot.cmd = mot_move;
    mot.speedcmd = speed;
    mot.dist = dist;
    return 0;
  }
  else
    return mot.finished;
}

int turn(double angle, double speed, int time)
{
  if (time == 0)
  {
    mot.cmd = mot_turn;
    mot.speedcmd = speed;
    mot.angle = angle;
    return 0;
  }
  else
    return mot.finished;
}

int follow_black_l(double speed, double dist, int color, int time)
{
  if (time == 0)
  {
    mot.cmd = mot_follow_black_l;
    mot.dist = dist;
    mot.speedcmd = speed;
    mot.color = color;
    return 0;
  }
  else
    return mot.finished;
}

int follow_black_r(double speed, double dist, int color, int time)
{
  if (time == 0)
  {
    mot.cmd = mot_follow_black_r;
    mot.dist = dist;
    mot.speedcmd = speed;
    mot.color = color;
    return 0;
  }
  else
    return mot.finished;
}

void sm_update(smtype *p)
{
  if (p->state != p->oldstate)
  {
    p->time = 0;
    p->oldstate = p->state;
  }
  else
  {
    p->time++;
  }
}

int crossline(int i, double color, double *data){
  if(*data == color){
    if(--i > 0){
      if(crossline(i, color, ++data)){
        return 1;
      }
    }
    else{
      return 1;
    }
  }
  return 0;
}

void calibrate_linesensor(symTableElement *linesensor, linesensortype *line){
  for(int i = 0; i < linesensor->length; i++){
    line->line_raw[i] = linesensor->data[i];
  }
  for(int i = 0; i < linesensor->length; i++){
    line->line_calibrate[i] = (line->line_raw[i] - BLACK_VALUE) / (WHITE_VALUE - BLACK_VALUE);
    if(line->line_calibrate[i] < BLACK_LINE_FOUND_VALUE){
      line->line_calibrate[i] = 0;
    }
    else if(line->line_calibrate[i] > WHITE_LINE_FOUND_VALUE){
      line->line_calibrate[i] = 1;
    }
  }
}

void update_linesensor(symTableElement *linesensor, linesensortype *line, double w)
{
  line->length = linesensor->length;
  calibrate_linesensor(linesensor, line);
  double *right = &line->line_calibrate[0];
  double *left = &line->line_calibrate[linesensor->length - 1];
  int right_pos = 0, left_pos = linesensor->length - 1, right_pos_white = 0, left_pos_white = linesensor->length - 1;
  line->find_l = line->find_r = 0;
  line->crossline = crossline(linesensor->length, 0.0, line->line_calibrate);
  line->find_l_white = line->find_r_white = 0;
  line->crossline_white = crossline(linesensor->length, 1.0, line->line_calibrate);
  for (int i = 0; i < linesensor->length; i++)
  {
    if (*right == 0.0)
    {
      line->find_r = 1;
      right_pos = i;
      break;
    }
    else
    {
      right++;
    }
  }
  for (int i = 0; i < linesensor->length; i++)
  {
    if (*left == 0.0)
    {
      line->find_l = 1;
      left_pos = linesensor->length - i - 1;
      break;
    }
    else
    {
      left--;
    }
  }
  right = &line->line_calibrate[0];
  left = &line->line_calibrate[linesensor->length - 1];
  for (int i = 0; i < linesensor->length; i++)
  {
    if (*right == 1.0)
    {
      line->find_r_white = 1;
      right_pos_white = i;
      break;
    }
    else
    {
      right++;
    }
  }
  for (int i = 0; i < linesensor->length; i++)
  {
    if (*left == 1.0)
    {
      line->find_l_white = 1;
      left_pos_white = linesensor->length - i - 1;
      break;
    }
    else
    {
      left--;
    }
  }
  line->left = left_pos;
  line->right = right_pos;
  line->left_white = left_pos_white;
  line->right_white = right_pos_white;
  calPos_linesensor(line, w);
}

void calPos_linesensor(linesensortype *line, double w)
{
  double leftpos = w, rightpos = 0.0, middlepos = w / 2.0, delta_sensor = w / (line->length);
  line->left_pos = delta_sensor * (line->left) - middlepos;
  line->right_pos = delta_sensor * (line->right) - middlepos;
  line->left_pos_white = delta_sensor * (line->left_white) - middlepos;
  line->right_pos_white = delta_sensor * (line->right_white) - middlepos;
}

void square(smtype *p, double dist, double direction, double speed){
  p->state = ms_init;
  p->state_index = 0;
  p->oldstate = -1;
  p->states_set[1] = ms_fwd;
  p->dist[1] = dist;
  p->speed[1] = speed;
  p->states_set[2] = ms_turn;
  p->angle[2] = direction * -90.0 / 180 * M_PI;
  p->speed[2] = speed;
  p->states_set[3] = ms_fwd;
  p->dist[3] = dist;
  p->speed[3] = speed;
  p->states_set[4] = ms_turn;
  p->angle[4] = direction * -90.0 / 180 * M_PI;
  p->speed[4] = speed;
  p->states_set[5] = ms_fwd;
  p->dist[5] = dist;
  p->speed[5] = speed;
  p->states_set[6] = ms_turn;
  p->angle[6] = direction * -90.0 / 180 * M_PI;
  p->speed[6] = speed;
  p->states_set[7] = ms_fwd;
  p->dist[7] = dist;
  p->speed[7] = speed;
  p->states_set[8] = ms_turn;
  p->angle[8] = direction * -90.0 / 180 * M_PI;
  p->speed[8] = speed;
  p->states_set[9] = ms_end;
}

void mission_fwd(smtype *p, int i, double dist, double speed){
  p->states_set[i] = ms_fwd;
  p->dist[i] = dist;
  p->speed[i] = speed;
}

void mission_turn(smtype *p, int i, double angle, double speed){
  angle = angle / 180 * M_PI;
  p->states_set[i] = ms_turn;
  p->angle[i] = angle;
  p->speed[i] = speed;
}

void mission_follow_black_l_line(smtype *p, int i, double speed, double dist, int color){
  p->states_set[i] = ms_follow_black_l;
  p->dist[i] = dist;
  p->speed[i] = speed;
  p->color[i] = color;
}

void mission_follow_black_r_line(smtype *p, int i, double speed, double dist, int color){
  p->states_set[i] = ms_follow_black_r;
  p->dist[i] = dist;
  p->speed[i] = speed;
  p->color[i] = color;
}

void mission_1(smtype *p){
  int i = 1;
  p->state = ms_init;
  p->state_index = 0;
  p->oldstate = -1;
  mission_follow_black_l_line(p, i++, 0.3, 10, 0);
  mission_fwd(p, i++, 0.23, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 10, 0);
  mission_fwd(p, i++, -1.0, 0.3);
  mission_turn(p, i++, 180.0, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 10, 0);
  mission_fwd(p, i++, 0.23, 0.3);
  mission_turn(p, i++, 90.0, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 10, 0);
  mission_fwd(p, i++, 0.23, 0.3);
  mission_turn(p, i++, 90.0, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 10, 0);
  mission_fwd(p, i++, 0.23, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 10, 0);
  mission_fwd(p, i++, 0.23, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 10, 0);
  mission_fwd(p, i++, 0.3, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 10, 0);
  mission_fwd(p, i++, 0.23, 0.3);

  // mission_fwd(p, i++, 2.2, 0.3);
  // mission_turn(p, i++, 90.0, 0.3);
  // mission_fwd(p, i++, 0.4, 0.3);
  // mission_turn(p, i++, -90.0, 0.3);
  // mission_follow_black_r_line(p, i++, 0.3, 10, 1);
  p->states_set[i++] = ms_end;
}