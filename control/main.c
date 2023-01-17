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
#include "constants.h"
#include "control.h"

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


// SMR input/output data

symTableElement *inputtable, *outputtable;
symTableElement *lenc, *renc, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl, *tick;

odotype odo;
smtype mission;
motiontype mot;
linesensortype line;
irsensortype ir;
lasersensortype laser;

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
  tick = getinputref("tick", inputtable);

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
      len=sprintf(buf,"scanpush cmd='zoneobst'\n");
      // len = sprintf(buf, "push  t=0.2 cmd='mrcobst width=0.4'\n");
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
  ir.length = irsensor->length - 1; // The last one is not used
  uint8_t i;
  for (i = 0; i < ir.length; i++) {
    ir.kA[i] = kA(i);
    ir.kB[i] = kB(i);
  }
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w = odo.w;
  running = 1;
  // square(&mission, 1.0, 1.0, 0.3);
  mission_1(&mission, &odo);
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
      mission.state_index++;
      mission.state = mission.states_set[mission.state_index];
      break;

    case ms_fwd:

      if (fwd(&mot, mission.dist[mission.state_index], mission.speed[mission.state_index], mission.time)){
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_fwd_black_stop:

      if (fwd(&mot, mission.dist[mission.state_index], mission.speed[mission.state_index], mission.time) || line.find_l || line.find_r){
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_fwd_Nonblack_stop:

      if (fwd(&mot, mission.dist[mission.state_index], mission.speed[mission.state_index], mission.time) || (!(line.find_l)) || (!(line.find_r))){
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
        printf("distance: %f ", laser.value[3], laser.value[4]);
        FILE *fp;
        fp = fopen("distance.dat", "w");
        for (int i = 0; i < 9; i++)
        {
          fprintf(fp, "%f ", laser.value[i]);
        }
        fprintf(fp, "%f ", laser.min);
        fprintf(fp, "\n");
        fclose(fp);
      }
      break;

    case ms_fwd_wall_stop:

      if (fwd(&mot, mission.dist[mission.state_index], mission.speed[mission.state_index], mission.time) || laser.value[4] < mission.threshold[mission.state_index]){
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_fwd_findgate:

      if (fwd(&mot, mission.dist[mission.state_index], mission.speed[mission.state_index], mission.time) || laser.value[8] < mission.threshold[mission.state_index]){
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_fwd_findgate_stop:

      if (fwd(&mot, mission.dist[mission.state_index], mission.speed[mission.state_index], mission.time) || laser.value[8] > mission.threshold[mission.state_index]){
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_fwd_findgate_l:

      if (fwd(&mot, mission.dist[mission.state_index], mission.speed[mission.state_index], mission.time) || laser.value[0] < mission.threshold[mission.state_index]){
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_fwd_findgate_stop_l:

      if (fwd(&mot, mission.dist[mission.state_index], mission.speed[mission.state_index], mission.time) || laser.value[0] > mission.threshold[mission.state_index]){
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_turn:
      if (turn(&mot, mission.angle[mission.state_index], mission.speed[mission.state_index], mission.time)){
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_turn_find_black:
      if (turn(&mot, mission.angle[mission.state_index], mission.speed[mission.state_index], mission.time) || line.line_calibrate[2] < BLACK_LINE_FOUND_VALUE){
        if(line.find_r){
          printf("find black line");
        }
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_follow_black_l:
      if (follow_black_l(&mot, mission.speed[mission.state_index], mission.dist[mission.state_index], mission.color[mission.state_index], mission.time) || (!(line.find_l) && !(line.find_l_old)) || line.crossline)
      {
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_follow_black_l_gate_1:
      if (follow_black_l(&mot, mission.speed[mission.state_index], mission.dist[mission.state_index], mission.color[mission.state_index], mission.time) || laser.value[0] < mission.gate_threshold[mission.state_index])
      {
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
        mission.gate_pos_1 = odo.x;
      }
      break;

    case ms_follow_black_l_gate_2:
      if (follow_black_l(&mot, mission.speed[mission.state_index], mission.dist[mission.state_index], mission.color[mission.state_index], mission.time) || laser.value[0] > mission.gate_threshold[mission.state_index])
      {
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
        mission.gate_pos_2 = odo.x;
        mission.gate_pos = fabs(mission.gate_pos_2 - mission.gate_pos_1) / 2;
      }
      break;

    case ms_follow_black_r:
      if (follow_black_r(&mot, mission.speed[mission.state_index], mission.dist[mission.state_index], mission.color[mission.state_index], mission.time) || (!(line.find_r) && !(line.find_r_old)) || line.crossline)
      {
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_follow_black_r_wall_stop:
      if (follow_black_r(&mot, mission.speed[mission.state_index], mission.dist[mission.state_index], mission.color[mission.state_index], mission.time) || laser.value[4] < mission.threshold[mission.state_index])
      {
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_follow_white:
      if (follow_black_l(&mot, mission.speed[mission.state_index], mission.dist[mission.state_index], mission.color[mission.state_index], mission.time) || line.crossline)
      {
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_follow_wall_l:
      if (follow_wall_l(&mot, mission.speed[mission.state_index], mission.dist[mission.state_index], mission.dist_fromWall[mission.state_index], mission.time))
      {
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_follow_wall_r:
      if (follow_wall_r(&mot, mission.speed[mission.state_index], mission.dist[mission.state_index], mission.dist_fromWall[mission.state_index], mission.time))
      {
        mission.state_index++;
        mission.state = mission.states_set[mission.state_index];
      }
      break;

    case ms_wait_1s:;
      static int wait_timer;
      if (mission.time == 0)
        wait_timer = *tick->data;
      else if (*tick->data - wait_timer >= 100) // Wait 1s
        mission.state = mission.states_set[++mission.state_index];
      break;

    case ms_end:
      mot.cmd = mot_stop;
      running = 0;
      break;
    }
    /*  end of mission  */

    mot.left_pos = odo.left_pos;
    mot.right_pos = odo.right_pos;
    update_motcon(&mot, &odo, &line, &ir, &laser);
    speedl->data[0] = 100 * mot.motorspeed_l;
    speedl->updated = 1;
    speedr->data[0] = 100 * mot.motorspeed_r;
    speedr->updated = 1;

    line.find_l_old  = line.find_l;
    line.find_r_old = line.find_r;
    update_linesensor(linesensor, &line, odo.w);
    updateIRSensor(irsensor, &ir);
    updateLaserSensor(&laser, laserpar);
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
        fprintf(fp, "%f ", line.line_calibrate_raw[i]);
      }
      fprintf(fp, "%d ", line.left);
      fprintf(fp, "%d ", line.right);
      fprintf(fp, "%f ", line.left_pos);
      fprintf(fp, "%f ", line.right_pos);
      fprintf(fp, "%d ", line.find_l);
      fprintf(fp, "%d ", line.maxIndex);
      fprintf(fp, "%f ", line.left_pos_white);
      fprintf(fp, "%f ", line.right_pos_white);
      fprintf(fp, "%d ", line.find_l_white);
      fprintf(fp, "%f ", line.max - line.min);
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
        fprintf(fp, "%f ", line.line_calibrate_raw[i]);
      }
      fprintf(fp, "%d ", line.left);
      fprintf(fp, "%d ", line.right);
      fprintf(fp, "%f ", line.left_pos);
      fprintf(fp, "%f ", line.right_pos);
      fprintf(fp, "%d ", line.find_l);
      fprintf(fp, "%d ", line.maxIndex);
      fprintf(fp, "%f ", line.left_pos_white);
      fprintf(fp, "%f ", line.right_pos_white);
      fprintf(fp, "%d ", line.find_l_white);
      fprintf(fp, "%f ", line.max - line.min);
      fprintf(fp, "\n");
      fclose(fp);
    }
    
    if(logcount == 0){
      FILE *fp;
      fp = fopen("irsensorlog.dat", "w");
      for (int i = 0; i < ir.length; i++)
      {
        fprintf(fp, "%f ", ir.value[i]);
      }
      fprintf(fp, "\n");
      fclose(fp);
    }
    else{
      FILE *fp;
      fp = fopen("irsensorlog.dat", "a");
      for (int i = 0; i < ir.length; i++)
      {
        fprintf(fp, "%f ", ir.value[i]);
      }
      fprintf(fp, "\n");
      fclose(fp);
    }
    
    if(logcount == 0){
      FILE *fp;
      fp = fopen("PID.dat", "w");
      fprintf(fp, "%d ", logcount);
      fprintf(fp, "%f ", line.left_pos_white);
      fprintf(fp, "\n");
      fclose(fp);
    }
    else{
      FILE *fp;
      fp = fopen("PID.dat", "a");
      fprintf(fp, "%d ", logcount);
      fprintf(fp, "%f ", line.left_pos_white);
      fprintf(fp, "\n");
      fclose(fp);
    }

    if(logcount == 0){
      FILE *fp;
      fp = fopen("lasersensorlog.dat", "w");
      for (int i = 0; i < 9; i++)
      {
        fprintf(fp, "%f ", laser.value[i]);
      }
      fprintf(fp, "%f ", laser.min);
      fprintf(fp, "\n");
      fclose(fp);
    }
    else{
      FILE *fp;
      fp = fopen("lasersensorlog.dat", "a");
      for (int i = 0; i < 9; i++)
      {
        fprintf(fp, "%f ", laser.value[i]);
      }
      fprintf(fp, "%f ", laser.min);
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