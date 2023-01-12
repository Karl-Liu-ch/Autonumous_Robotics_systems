#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "rhd.h"
#include "constants.h"
#include "control.h"

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

void update_motcon(motiontype *p, odotype *q, linesensortype *line, irsensortype *ir, lasersensortype *laser)
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
    case mot_follow_black_r:
    case mot_follow_wall_l:
    case mot_follow_wall_r:
      p->startpos = (p->left_pos + p->right_pos) / 2;
      p->curcmd = p->cmd;
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

  case mot_follow_wall_l:
    if (laser->min < 0.4 && fabs((p->right_pos + p->left_pos) / 2 - p->startpos) < fabs(p->dist))
    {
      double error;
      error = (laser->min - p->dist_fromWall) * fabs(p->speedcmd);
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

  case mot_follow_wall_r:
    if (laser->min < 0.4 && fabs((p->right_pos + p->left_pos) / 2 - p->startpos) < fabs(p->dist))
    {
      double error;
      error = (laser->min - p->dist_fromWall) * fabs(p->speedcmd);
      double error_black_D = error - q->error_old;
      double direction = fabs(p->speedcmd) / p->speedcmd;
      q->error_all += error;
      q->error_old = error;
      p->motorspeed_l_old = p->motorspeed_l;
      p->motorspeed_r_old = p->motorspeed_r;
      p->motorspeed_l = direction * (fabs(p->speedcmd) + KP * error + KI * q->error_all + KD * error_black_D);
      p->motorspeed_r = direction * (fabs(p->speedcmd) - KP * error - KI * q->error_all - KD * error_black_D);
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

int fwd(motiontype *mot, double dist, double speed, int time)
{
  if (time == 0)
  {
    mot->cmd = mot_move;
    mot->speedcmd = speed;
    mot->dist = dist;
    return 0;
  }
  else
    return mot->finished;
}

int turn(motiontype *mot, double angle, double speed, int time)
{
  if (time == 0)
  {
    mot->cmd = mot_turn;
    mot->speedcmd = speed;
    mot->angle = angle;
    return 0;
  }
  else
    return mot->finished;
}

int follow_black_l(motiontype *mot, double speed, double dist, double color, int time)
{
  if (time == 0)
  {
    mot->cmd = mot_follow_black_l;
    mot->dist = dist;
    mot->speedcmd = speed;
    mot->color = color;
    return 0;
  }
  else
    return mot->finished;
}

int follow_black_r(motiontype *mot, double speed, double dist, double color, int time)
{
  if (time == 0)
  {
    mot->cmd = mot_follow_black_r;
    mot->dist = dist;
    mot->speedcmd = speed;
    mot->color = color;
    return 0;
  }
  else
    return mot->finished;
}

int follow_wall_l(motiontype *mot, double speed, double dist, double dist_fromWall, int time)
{
  if (time == 0)
  {
    mot->cmd = mot_follow_wall_l;
    mot->dist = dist;
    mot->speedcmd = speed;
    mot->dist_fromWall = dist_fromWall;
    return 0;
  }
  else
    return mot->finished;
}

int follow_wall_r(motiontype *mot, double speed, double dist, double dist_fromWall, int time)
{
  if (time == 0)
  {
    mot->cmd = mot_follow_wall_r;
    mot->dist = dist;
    mot->speedcmd = speed;
    mot->dist_fromWall = dist_fromWall;
    return 0;
  }
  else
    return mot->finished;
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
  line->find_l_white = line->find_r_white = 0;
  for(int i = 0; i < linesensor->length; i++){
    line->line_raw[i] = linesensor->data[i];
  }
  for(int i = 0; i < linesensor->length; i++){
    line->line_calibrate[i]  = line->line_calibrate_raw[i] = (line->line_raw[i] - BLACK_VALUE(i)) / (WHITE_VALUE(i) - BLACK_VALUE(i));
    if(line->line_calibrate[i] < BLACK_LINE_FOUND_VALUE){
      line->line_calibrate[i] = 0;
    }
    else if(line->line_calibrate[i] > WHITE_LINE_FOUND_VALUE){
      line->line_calibrate[i] = 1;
    }
  }
  line->max = 0.0;
  line->min = 1.0;
  for(int i = 0; i < linesensor->length; i++){
    if(line->line_calibrate_raw[i] > line->max){
      line->max = line->line_calibrate_raw[i];
      line->maxIndex = i;
    }
    if(line->line_calibrate_raw[i] < line->min){
      line->min = line->line_calibrate_raw[i];
      line->minIndex = i;
    }
  }
  if(line->max - line->min >= 0.04){
    line->find_l_white = line->find_r_white = 1;
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
  // line->left_white = left_pos_white;
  // line->right_white = right_pos_white;
  calPos_linesensor(line, w);
}

void calPos_linesensor(linesensortype *line, double w)
{
  double leftpos = w, rightpos = 0.0, middlepos = w / 2.0, delta_sensor = w / (line->length);
  line->left_pos = delta_sensor * (line->left) - middlepos;
  line->right_pos = delta_sensor * (line->right) - middlepos;
  // line->left_pos_white = delta_sensor * (line->left_white) - middlepos;
  // line->right_pos_white = delta_sensor * (line->right_white) - middlepos;
  // line->left_pos = delta_sensor * (line->minIndex) - middlepos;
  // line->right_pos = delta_sensor * (line->minIndex) - middlepos;
  line->left_pos_white = delta_sensor * (line->maxIndex) - middlepos;
  line->right_pos_white = delta_sensor * (line->maxIndex) - middlepos;
}

void updateIRSensor(symTableElement *irsensor, irsensortype *p) {
  uint8_t i;
  for (i = 0; i < p->length; i++) {
    p->value_raw[i] = irsensor->data[i]; // First sensor is at the left side and so on
    if (p->value_raw[i] > IR_MIN_VALUE(i)) // If this is below this value, then we can trust the measurement
      p->value[i] = p->kA[i] / (p->value_raw[i] - p->kB[i]);
    else
      p->value[i] = 0.60; // This is the maximum range of the ir sensors by specs
  }
}

void printIRSensor(irsensortype *p) {
  uint8_t i;
  printf("Values: ");
  for (i = 0; i < p->length; i++)
    printf("%f ", p->value[i]);
    //printf("%d ", p->value_raw[i]);
  printf("\n");
}

void updateLaserSensor(lasersensortype *p, double *q){
  p->min = 1000.0;
  for(int i = 0; i < 9; i++){
    p->value[i] = *q++;
    if(p->value[i] < p->min){
      p->min = p->value[i];
    }
  }
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

void mission_fwd_black_stop(smtype *p, int i, double dist, double speed){
  p->states_set[i] = ms_fwd_black_stop;
  p->dist[i] = dist;
  p->speed[i] = speed;
}

void mission_fwd_Nonblack_stop(smtype *p, int i, double dist, double speed){
  p->states_set[i] = ms_fwd_Nonblack_stop;
  p->dist[i] = dist;
  p->speed[i] = speed;
}

void mission_fwd_wall_stop(smtype *p, int i, double dist, double speed, double threshold){
  p->states_set[i] = ms_fwd_wall_stop;
  p->dist[i] = dist;
  p->speed[i] = speed;
  p->threshold[i] = threshold;
}

void mission_turn(smtype *p, int i, double angle, double speed){
  angle = angle / 180 * M_PI;
  p->states_set[i] = ms_turn;
  p->angle[i] = angle;
  p->speed[i] = speed;
}

void mission_follow_black_l_line(smtype *p, int i, double speed, double dist, double color){
  p->states_set[i] = ms_follow_black_l;
  p->dist[i] = dist;
  p->speed[i] = speed;
  p->color[i] = color;
}

void mission_follow_black_l_line_gate_1(smtype *p, int i, double speed, double dist, double color, double gate_threshold){
  p->states_set[i] = ms_follow_black_l_gate_1;
  p->dist[i] = dist;
  p->speed[i] = speed;
  p->color[i] = color;
  p->gate_threshold[i] = gate_threshold;
}

void mission_follow_black_l_line_gate_2(smtype *p, int i, double speed, double dist, double color, double gate_threshold){
  p->states_set[i] = ms_follow_black_l_gate_2;
  p->dist[i] = dist;
  p->speed[i] = speed;
  p->color[i] = color;
  p->gate_threshold[i] = gate_threshold;
}

void mission_follow_black_r_line(smtype *p, int i, double speed, double dist, double color){
  p->states_set[i] = ms_follow_black_r;
  p->dist[i] = dist;
  p->speed[i] = speed;
  p->color[i] = color;
}

void mission_follow_wall_l(smtype *p, int i, double speed, double dist, double dist_fromWall){
  p->states_set[i] = ms_follow_wall_l;
  p->dist[i] = dist;
  p->speed[i] = speed;
  p->dist_fromWall[i] = dist_fromWall;
}

void mission_follow_wall_r(smtype *p, int i, double speed, double dist, double dist_fromWall){
  p->states_set[i] = ms_follow_wall_r;
  p->dist[i] = dist;
  p->speed[i] = speed;
  p->dist_fromWall[i] = dist_fromWall;
}

void mission_wait_1s(smtype *p, int i){
  p->states_set[i] = ms_wait_1s;
}

// void mission_wait_1s(smtype *p, int i, lasersensortype *laser){
//   p->states_set[i] = ms_wait_1s;
//   FILE *fp;
//   fp = fopen("distance.dat", "w");
//   for (int i = 0; i < 9; i++)
//   {
//     fprintf(fp, "%f ", laser->value[i]);
//   }
//   fprintf(fp, "%f ", laser->min);
//   fprintf(fp, "\n");
//   fclose(fp);
// }

void mission_1(smtype *p, odotype *q){
  int i = 1;
  p->state = ms_init;
  p->state_index = 0;
  p->oldstate = -1;
  // distance test
  mission_fwd(p, i++, 0.6, 0.3);
  mission_turn(p, i++, -90.0, 0.3);
  mission_fwd_Nonblack_stop(p, i++, -1, 0.3);
  
  mission_fwd(p, i++, 0.1, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 10, 0);
  mission_fwd(p, i++, -0.7, 0.3);
  mission_turn(p, i++, -90.0, 0.3);
  mission_fwd_black_stop(p, i++, 1, 0.3);
  mission_fwd(p, i++, 0.23, 0.3);
  mission_turn(p, i++, 90.0, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 10, 0);
  mission_fwd(p, i++, 0.23, 0.3);
  mission_turn(p, i++, 90.0, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 10, 0); //go through the first gate
  mission_fwd(p, i++, 0.23, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 10, 0);
  mission_fwd(p, i++, 0.23, 0.3);
  mission_follow_black_l_line_gate_1(p, i++, 0.3, 10, 0, 0.4);
  mission_follow_black_l_line_gate_2(p, i++, 0.3, 10, 0, 0.4);
  mission_fwd(p, i++, 0.5 - p->gate_pos, 0.3);
  mission_turn(p, i++, 90.0, 0.3);
  mission_fwd(p, i++, 1, 0.3);
  mission_fwd_wall_stop(p, i++, 2, 0.3, 0.1);
  mission_turn(p, i++, 90.0, 0.3);
  mission_follow_wall_r(p, i++, 0.3, 10, 0.3);
  mission_fwd(p, i++, 0.5, 0.3);
  mission_turn(p, i++, -90.0, 0.3);
  mission_fwd(p, i++, 0.6, 0.3);
  mission_turn(p, i++, -90.0, 0.3);
  mission_follow_wall_r(p, i++, 0.3, 10, 0.3);
  mission_fwd(p, i++, 0.5, 0.3);
  mission_turn(p, i++, -90.0, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 0.6, 0);
  mission_turn(p, i++, 180.0, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 10, 0);
  mission_fwd(p, i++, 0.1, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 10, 0);
  
  mission_fwd(p, i++, 0.23, 0.3);
  mission_turn(p, i++, -90.0, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 10, 0);
  mission_fwd_wall_stop(p, i++, 2, 0.3, 0.1);
  mission_turn(p, i++, 90.0, 0.3);
  mission_follow_wall_r(p, i++, 0.3, 10, 0.3);
  mission_fwd(p, i++, 0.45, 0.3);
  mission_turn(p, i++, -90.0, 0.3);
  mission_fwd(p, i++, 0.45, 0.3);
  mission_turn(p, i++, -180.0, 0.3);
  mission_fwd(p, i++, 0.8, 0.3);
  mission_turn(p, i++, 90.0, 0.3);
  mission_fwd_black_stop(p, i++, 1, 0.3);
  mission_fwd(p, i++, 0.23, 0.3);
  mission_turn(p, i++, 90.0, 0.3);
  mission_follow_black_l_line(p, i++, 0.3, 10, 0);
  mission_fwd_wall_stop(p, i++, 2, 0.3, 0.05);
  p->states_set[i++] = ms_end;
}