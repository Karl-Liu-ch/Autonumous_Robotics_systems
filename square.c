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

#define ED 0.9999
#define WHEEL_DIAMETER_L 0.06522 * (1 + (1 - ED) / 2) /* m */
#define WHEEL_DIAMETER_R 0.06522 * (1 - (1 - ED) / 2)
#define WHEEL_SEPARATION 0.2596 /* m */
#define DELTA_M_L (M_PI * WHEEL_DIAMETER_L / 2000)
#define DELTA_M_R (M_PI * WHEEL_DIAMETER_R / 2000)
#define DEFAULT_ROBOTPORT 24902
#define EPSILON 0.01
#define K 2.0
#define KP 2.0
#define KI 2.0
#define KD 2.0

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
} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);

/********************************************
 * Motion control
 */

typedef struct
{ // input
  int cmd;
  int curcmd;
  double speedcmd;
  double dist;
  double start_theta;
  double angle;
  double left_pos, right_pos;
  double error_cur, error_old, error_all;
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
  mot_follow_black,
};

typedef struct
{
  int left, right, length, find_l, find_r, crossline;
  double left_pos, right_pos, middle_pos;
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
int follow_black(double speed, int time);

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

typedef struct
{
  int state, oldstate;
  int time;
} smtype;

void sm_update(smtype *p);

void update_linesensor(symTableElement *linesensor, linesensortype *line, double w);
void calibrate_linesensor(linesensortype *line, double w);

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
  ms_follow_black,
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
  mission.state = ms_init;
  mission.oldstate = -1;
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
      n = 4;
      dist = 1;
      angle = -90.0 / 180 * M_PI;
      // mission.state = ms_fwd;
      mission.state = ms_follow_black;
      break;

    case ms_fwd:
      if (fwd(dist, 0.4, mission.time))
        mission.state = ms_turn;
      break;

    case ms_turn:
      if (turn(angle, 0.4, mission.time))
      {
        n = n - 1;
        if (n == 0)
          mission.state = ms_end;
        else
          mission.state = ms_fwd;
      }
      break;

    case ms_follow_black:
      if (follow_black(0.3, mission.time))
      {
        mission.state = ms_end;
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
      fprintf(fp, "%d ", line.left);
      fprintf(fp, "%d ", line.right);
      fprintf(fp, "%f ", line.left_pos);
      fprintf(fp, "%f ", line.right_pos);
      fprintf(fp, "%d ", line.find_l);
      fprintf(fp, "%d ", line.crossline);
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
      fprintf(fp, "%d ", line.left);
      fprintf(fp, "%d ", line.right);
      fprintf(fp, "%f ", line.left_pos);
      fprintf(fp, "%f ", line.right_pos);
      fprintf(fp, "%d ", line.find_l);
      fprintf(fp, "%d ", line.crossline);
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

    case mot_follow_black:
      p->curcmd = mot_follow_black;
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
  double error_black_I = 0;
  double error_old = 0;
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
      p->motorspeed_l = direction * accelerate_speed(fabs(p->motorspeed_l_old), p->speedcmd, dist, 0.5);
      p->motorspeed_r = direction * accelerate_speed(fabs(p->motorspeed_r_old), p->speedcmd, dist, 0.5);
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
      p->motorspeed_r = direction * accelerate_speed(fabs(p->motorspeed_r_old), p->speedcmd, dist, 0.5);
      p->motorspeed_l = -direction * accelerate_speed(fabs(p->motorspeed_r_old), p->speedcmd, dist, 0.5);
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

  case mot_follow_black:
    if (line->find_l && (!(line->crossline)))
    {
      double error = line->left_pos;
      double error_black_D = error - error_old;
      error_black_I += error;
      error_old = error;
      p->motorspeed_l_old = p->motorspeed_l;
      p->motorspeed_r_old = p->motorspeed_r;
      p->motorspeed_r = accelerate_speed(p->motorspeed_r_old, p->speedcmd + KP * error + KI * error_black_I + KD * error_black_D, 10, 0.5);
      p->motorspeed_l = accelerate_speed(p->motorspeed_r_old, p->speedcmd - KP * error - KI * error_black_I - KD * error_black_D, 10, 0.5);
    }
    else
    {
      p->motorspeed_l = 0;
      p->motorspeed_r = 0;
      p->finished = 1;
    }
    break;

    // case mot_turn:
    //   if (p->angle > 0)
    //   {
    //     // if ((p->right_pos-p->startpos) < 0.5*p->angle*p->w){
    //     if ((p->angle + p->start_theta - q->theta) > 0)
    //     {
    //       p->motorspeed_l=-p->speedcmd;
    //       p->motorspeed_r=p->speedcmd;
    //       // exercise 3.6
    //       // double dist = 0.5 * (p->angle - (q->theta - p->start_theta)) * p->w;
    //       // p->motorspeed_l_old = p->motorspeed_l;
    //       // p->motorspeed_r_old = p->motorspeed_r;
    //       // p->motorspeed_r = accelerate_speed(p->motorspeed_r_old, p->speedcmd + angular_control(p->start_theta + p->angle, q->theta, K, p->w), dist, 0.5);
    //       // p->motorspeed_l = -accelerate_speed(p->motorspeed_r_old, p->speedcmd - angular_control(p->start_theta + p->angle, q->theta, K, p->w), dist, 0.5);
    //     }
    //     else
    //     {
    //       p->motorspeed_l = 0;
    //       p->motorspeed_r = 0;
    //       p->finished = 1;
    //     }
    //   }
    //   else
    //   {
    //     // if (p->left_pos-p->startpos < 0.5*fabs(p->angle)*p->w){
    //     if (fabs(p->angle + p->start_theta - q->theta) > 0)
    //     {
    //       p->motorspeed_l=p->speedcmd;
    //       p->motorspeed_r=-p->speedcmd;
    //       // exercise 3.6
    //       // double dist = 0.5 * (p->angle + p->start_theta - q->theta) * p->w;
    //       // p->motorspeed_l_old = p->motorspeed_l;
    //       // p->motorspeed_r_old = p->motorspeed_r;
    //       // p->motorspeed_l = accelerate_speed(p->motorspeed_l_old, p->speedcmd, dist, 0.5);
    //       // p->motorspeed_r = -p->motorspeed_l;
    //     }
    //     else
    //     {
    //       p->motorspeed_r = 0;
    //       p->motorspeed_l = 0;
    //       p->finished = 1;
    //     }
    //   }

    //   break;
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

int follow_black(double speed, int time)
{
  if (time == 0)
  {
    mot.cmd = mot_follow_black;
    mot.speedcmd = speed;
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

void update_linesensor(symTableElement *linesensor, linesensortype *line, double w)
{
  int crossline(int i, int *data){
    if(*data == 0){
      if(--i > 0){
        if(crossline(i, ++data)){
          return 1;
        }
      }
      else{
        return 1;
      }
    }
    return 0;
  }
  line->length = linesensor->length;
  int *right = &linesensor->data[0];
  int *left = &linesensor->data[linesensor->length - 1];
  int *middle = &linesensor->data[0];
  int *p, *q;
  int right_pos = 0, left_pos = linesensor->length - 1;
  p = right;
  q = left;
  line->find_l = line->find_r = 0;
  line->crossline = crossline(linesensor->length, linesensor->data);
  for (int i = 0; i < linesensor->length; i++)
  {
    if (*right == 0)
    {
      line->find_r = 1;
    }
    if (*p < *right)
    {
      right = p;
      right_pos = i;
      line->find_r = 1;
    }
    else
    {
      p++;
    }
    if (*left == 0)
    {
      line->find_l = 1;
    }
    if (*q < *left)
    {
      left = q;
      left_pos = linesensor->length - i - 1;
      line->find_l = 1;
    }
    else
    {
      q--;
    }
  }
  line->left = left_pos;
  line->right = right_pos;
  calibrate_linesensor(line, w);
}

void calibrate_linesensor(linesensortype *line, double w)
{
  double leftpos = w, rightpos = 0.0, middlepos = w / 2.0, delta_sensor = w / (line->length);
  line->left_pos = delta_sensor * (line->left) - middlepos;
  line->right_pos = delta_sensor * (line->right) - middlepos;
}