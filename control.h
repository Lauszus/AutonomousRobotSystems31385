#ifndef _control_h_
#define _control_h_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "rhd.h"

#include "constants.h"

enum { mot_stop = 1, mot_move, mot_move_bwd, mot_follow_black, mot_turn };
double dUl, dUr, dU, dPhi, x, y, phi;

typedef struct { // Input signals
  int left_enc,right_enc; // encoderticks
  // parameters
  double w; // wheel separation
  double cr,cl; // meters per encodertick
  // Output signals
  double right_pos,left_pos;
  // internal variables
  int left_enc_old, right_enc_old;
  // Difference between current pos and new measurement
  int left_delta_pos, right_delta_pos;
} odotype;

typedef struct { // Input
  int cmd;
  int curcmd;
  double speedcmd;
  double dist;
  double angle, startAngle;
  double left_pos, right_pos;
  // parameters
  double w;
  // output
  double motorspeed_l, motorspeed_r;
  int finished;
  // internal variables
  double startpos;
  double x0, y0; // Start coordinates
} motiontype;

typedef struct {
  int state, oldstate;
  int programState[100];
  double dist[100];
  double speed[100];
  int time;
} smtype;

typedef struct {
  int32_t value_raw[8];
  double value[8];
  uint8_t length;
  double lowest_val, lowest_val_left, lowest_val_right;
  uint8_t lowest_pos;
  uint8_t black_line_found;
  double center_mass, center_mass_neighbors;
} linesensortype;

typedef struct {
  int32_t value_raw[5];
  double value[5];
  double kA[5], kB[5]; // Calibration values
  uint8_t length;
  uint8_t ignoreObs; // Ignore obstacles
} irsensortype;

void updateIRSensor(symTableElement *irsensor, irsensortype *p);
void printIRSensor(irsensortype *p);
#if CALIBRATE_IR_SENSOR_FRONT
uint8_t calibrateIRSensorFront(irsensortype *p);
#endif
#if CALIBRATE_IR_SENSOR_LEFT
uint8_t calibrateIRSensorLeft(irsensortype *p);
#endif
#if CALIBRATE_IR_SENSOR_RIGHT
uint8_t calibrateIRSensorRight(irsensortype *p);
#endif
void updateLineSensor(symTableElement *linesensor, linesensortype *p);
void printLineSensor(linesensortype *p);
#if CALIBRATE_LINE_SENSOR
uint8_t calibrateLineSensor(linesensortype *p);
#endif
void update_pos(odotype *p);
void reset_odo(odotype *p);
void update_odo(odotype *p);
void update_motcon(motiontype *p, linesensortype *line);
int followBlackLine(motiontype *mot, double dist, double speed, int time);
int fwd(motiontype *mot, double dist, double speed, int time);
int bwd(motiontype *mot, double dist, double speed, int time);
int turn(motiontype *mot, double angle, double speed, int time);
void sm_update(smtype *p);

#endif
