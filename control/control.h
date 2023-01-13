#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "rhd.h"
#include "constants.h"

enum
{
  ms_init,
  ms_fwd,
  ms_fwd_black_stop,
  ms_fwd_Nonblack_stop,
  ms_fwd_wall_stop,
  ms_fwd_findgate,
  ms_fwd_findgate_stop,
  ms_fwd_findgate_l,
  ms_fwd_findgate_stop_l,
  ms_turn,
  ms_follow_black_l,
  ms_follow_black_l_gate_1,
  ms_follow_black_l_gate_2,
  ms_follow_black_r,
  ms_follow_white,
  ms_follow_wall_l,
  ms_follow_wall_r,
  ms_wait_1s,
  ms_end
};
enum
{
  mot_stop = 1,
  mot_move,
  mot_turn,
  mot_follow_black_l,
  mot_follow_black_r,
  mot_follow_wall_l,
  mot_follow_wall_r,
};
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
  double dist_fromWall;
  int finished;
  // internal variables
  double startpos;
} motiontype;

typedef struct
{
  int left, right, length, find_l, find_r, crossline, left_white, right_white, find_l_white, find_r_white, crossline_white, maxIndex, minIndex;
  double left_pos, right_pos, left_pos_white, right_pos_white, max, min;
  double line_raw[8], line_calibrate[8], line_calibrate_raw[8];
} linesensortype;

typedef struct
{
  int state, oldstate;
  int states_set[1000];
  double dist[1000], angle[1000], speed[1000], color[1000], gate_threshold[1000], dist_fromWall[1000], threshold[1000];
  int state_index;
  int time;
  double gate_pos_1, gate_pos_2, gate_pos;
} smtype;

typedef struct {
  int32_t value_raw[5];
  double value[5];
  double kA[5], kB[5]; // Calibration values
  uint8_t length;
  uint8_t ignoreObs; // Ignore obstacles
} irsensortype;

typedef struct {
  double value[8], value_old[8], min;
} lasersensortype;

void updateLaserSensor(lasersensortype *p, double *q);
void updateIRSensor(symTableElement *irsensor, irsensortype *p);
void printIRSensor(irsensortype *p);
void update_motcon(motiontype *p, odotype *q, linesensortype *line, irsensortype *ir, lasersensortype *laser);
int fwd(motiontype *mot, double dist, double speed, int time);
int turn(motiontype *mot, double angle, double speed, int time);
int follow_black_l(motiontype *mot, double speed, double dist, double color, int time);
int follow_black_r(motiontype *mot, double speed, double dist, double color, int time);
int follow_wall_l(motiontype *mot, double speed, double dist, double dist_fromWall, int time);
int follow_wall_r(motiontype *mot, double speed, double dist, double dist_fromWall, int time);
void printState(int state);
void sm_update(smtype *p);
int crossline(int i, double color, double *data);
void update_linesensor(symTableElement *linesensor, linesensortype *line, double w);
void calPos_linesensor(linesensortype *line, double w);
// void square(smtype *p, double dist, double direction, double speed);
// void mission_fwd(smtype *p, int i, double dist, double speed);
// void mission_fwd_black_stop(smtype *p, int i, double dist, double speed);
// void mission_turn(smtype *p, int i, double angle, double speed);
// void mission_follow_black_l_line(smtype *p, int i, double speed, double dist, double color);
// void mission_follow_black_l_line_gate_1(smtype *p, int i, double speed, double dist, double color, double gate_threshold);
// void mission_follow_black_l_line_gate_2(smtype *p, int i, double speed, double dist, double color, double gate_threshold);
// void mission_follow_black_r_line(smtype *p, int i, double speed, double dist, double color);
// void mission_wait_1s(smtype *p, int i);
// void mission_1(smtype *p, odotype *q);
