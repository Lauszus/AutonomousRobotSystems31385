#include "control.h"

double dUl, dUr, dU, dPhi, x, y, phi;
double follow_error_old;

double getX() {
  return x / DIST_CAL;
}

double getXRaw() {
  return x;
}

double getY() {
  return y / DIST_CAL;
}

double getYRaw() {
  return y;
}

double getPhi() {
  return phi;
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

#if CALIBRATE_IR_SENSOR_FRONT
uint8_t calibrateIRSensorFront(irsensortype *p) { // Only reads the front IR sensors
  static uint8_t calArray[100][3], calIndex = 0;
  uint8_t i;
  for (i = 0; i < p->length - 2; i++)
    calArray[calIndex][i] = p->value_raw[i + 1];

  if (++calIndex >= 100) {
    char buf[1000];
    double avg[p->length - 2];
    memset(avg, 0, sizeof(avg));

    FILE *file = fopen("ir_cal.dat", "a");

    uint8_t j;
    for (i = 0; i < p->length - 2; i++) {
      for (j = 0; j < 100; j++)
	avg[i] += calArray[j][i];
      avg[i] /= 100;
    }

    for (i = 0; i < p->length - 2; i++) {
      sprintf(buf, "%f ", avg[i]);
      fwrite(buf, sizeof(buf[0]), strlen(buf), file);
      printf("%s", buf);
    }

    sprintf(buf, "\n");
    fwrite(buf, sizeof(buf[0]), strlen(buf), file);
    printf("%s", buf);

    fclose(file);
    return 1;
  }
  return 0;
}
#elif CALIBRATE_IR_SENSOR_LEFT || CALIBRATE_IR_SENSOR_RIGHT
#if CALIBRATE_IR_SENSOR_LEFT
uint8_t calibrateIRSensorLeft(irsensortype *p) {
#else
uint8_t calibrateIRSensorRight(irsensortype *p) {
#endif
  static uint8_t calArray[100], calIndex = 0;
#if CALIBRATE_IR_SENSOR_LEFT
  calArray[calIndex] = p->value_raw[0];
#else
  calArray[calIndex] = p->value_raw[4];
#endif

  if (++calIndex >= 100) {
    char buf[1000];
    double avg = 0;

#if CALIBRATE_IR_SENSOR_LEFT
    FILE *file = fopen("ir_cal_left.dat", "a");
#else
    FILE *file = fopen("ir_cal_right.dat", "a");
#endif

    uint8_t i;
    for (i = 0; i < 100; i++)
      avg += calArray[i];
    avg /= 100;


    sprintf(buf, "%f\n", avg);
    fwrite(buf, sizeof(buf[0]), strlen(buf), file);
    printf("%s", buf);
    fclose(file);
    return 1;
  }
  return 0;
}
#endif

void updateLineSensor(symTableElement *linesensor, linesensortype *p) {
  uint8_t i;
  for (i = 0; i < p->length; i++) {
    p->value_raw[i] = linesensor->data[i];
    p->value[i] = (BLACK_VALUE - (double)(p->value_raw[i])) / (BLACK_VALUE - WHITE_VALUE);
  }

  p->lowest_val = p->value[0];
  p->lowest_pos = 0;
  p->highest_val = p->value[0];
  p->highest_pos = 0;
  for (i = 1; i < p->length; i++) {
    if (p->value[i] < p->lowest_val) {
      p->lowest_val = p->value[i];
      p->lowest_pos = i;
    } else if (p->value[i] > p->highest_val) {
      p->highest_val = p->value[i];
      p->highest_pos = i;
    }
  }

  if (p->lowest_pos > 0) // Making sure the index of the lowest sensor reading is not out of bounds
    p->lowest_val_right = p->value[p->lowest_pos - 1];
  else
    p->lowest_val_right = -1;
  if (p->lowest_pos < p->length - 1)
    p->lowest_val_left = p->value[p->lowest_pos + 1];
  else
    p->lowest_val_left = -1;

  if (p->highest_pos > 0) // Making sure the index of the highest sensor reading is not out of bounds
    p->highest_val_right = p->value[p->highest_pos - 1];
  else
    p->highest_val_right = -1;
  if (p->highest_pos < p->length - 1)
    p->highest_val_left = p->value[p->highest_pos + 1];
  else
    p->highest_val_left = -1;

  for (i = 0; i <= 7; i++) { // Checking if we are crossing a black line
  	if (p->value[i] > BLACK_LINE_FOUND_VALUE)
  		break;
  }
  if (i == 8) {
    //printf("Black line found\n");
    p->black_line_found =1;
  }
  else
    p->black_line_found =0;

  for (i = 0; i <= 7; i++) { //Checking if we are crossing a white line
  	if (p->value[i] < WHITE_LINE_FOUND_VALUE)
  		break;
  }
  if (i == 8) {
    //printf("White line found\n");
    p->white_line_found =1;
  }
  else
    p->white_line_found =0;

  double sumTop = 0, sumBot = 0;
  // Finding the center of mass for the lowest sensor and the two neighbors on a black line
  for (i = (p->lowest_pos > 0 ? p->lowest_pos - 1 : p->lowest_pos); i <= (p->lowest_pos < p->length - 1 ? p->lowest_pos + 1 : p->lowest_pos); i++) {
  	double intensity =  1 - p->value[i];
  	sumTop += (i + 1) * intensity;
  	sumBot += intensity;
  }
  p->center_mass_neighbors[0] = sumTop / sumBot;

    // Finding the center of mass for all sensors on a black line
  sumTop = sumBot = 0;
  for (i = 0; i < p->length; i++) {
    double intensity = 1 - p->value[i];
    sumTop += (i + 1) * intensity;
    sumBot += intensity;
  }
  p->center_mass[0] = sumTop / sumBot;

  //Finding the center of mass for the lowest sensor and the two neighbors on a white line
  sumTop = sumBot = 0;
  for (i = (p->highest_pos > 0 ? p->highest_pos - 1 : p->highest_pos); i <= (p->highest_pos < p->length - 1 ? p->highest_pos + 1 : p->highest_pos); i++) {
    double intensity = p->value[i];
    sumTop += (i + 1) * intensity;
    sumBot += intensity;
  }
  p->center_mass_neighbors[1] = sumTop / sumBot;

  // Finding the center of mass for all sensors on a white line
  sumTop = sumBot = 0;
  for (i = 0; i < p->length; i++) {
    double intensity = p->value[i];
    sumTop += (i + 1) * intensity;
    sumBot += intensity;
  }
  p->center_mass[1] = sumTop / sumBot;
}

void printLineSensor(linesensortype *p) {
  uint8_t i;
  printf("Values: ");
  for (i = 0; i < p->length; i++)
    printf("%f ", p->value[i]);
  printf(" Low val (l,c,r): %f,%f,%f Pos: %d CM (B): %f %f CM (W): %f %f\n", p->lowest_val_left, p->lowest_val, p->lowest_val_right, p->lowest_pos, p->center_mass[0], p->center_mass_neighbors[0], p->center_mass[1], p->center_mass_neighbors[1]);
}

#if CALIBRATE_LINE_SENSOR
uint8_t calibrateLineSensor(linesensortype *p) {
  static uint8_t calArray[100][8], calIndex = 0;
  uint8_t i;
  for (i = 0; i < p->length; i++)
    calArray[calIndex][i] = p->value_raw[i];

  if (++calIndex >= 100) {
    char buf[1000];
    double avg[p->length];
    memset(avg, 0, sizeof(avg));

    FILE *file = fopen("lineCal_White.dat", "w");

    uint8_t j;
    for (i = 0; i < p->length; i++) {
      for (j = 0; j < 100; j++)
	avg[i] += calArray[j][i];
      avg[i] /= 100;
    }

    for (i = 0; i < p->length; i++) {
      sprintf(buf, "%f ", avg[i]);
      fwrite(buf, sizeof(buf[0]), strlen(buf), file);
      printf("%s", buf);
    }

    for (i = 1; i < p->length; i++)
      avg[0] += avg[i];
    sprintf(buf, "\n\nAverage: %f\n", avg[0] / p->length);
    fwrite(buf, sizeof(buf[0]), strlen(buf), file);
    printf("%s", buf);

    fclose(file);
    return 1;
  }
  return 0;
}
#endif

/*
char buf[1000000];
int length = 0;

void writeOutput() {
  printf("%s", buf);
  FILE *file = fopen("linesensor.dat", "w");
  fwrite(buf, sizeof(buf[0]), strlen(buf), file);
  fclose(file);
}
*/

void update_pos(odotype *p) {
  dUl = DELTA_M_L * p->left_delta_pos;
  dUr = DELTA_M_R * p->right_delta_pos;
  dU = (dUr + dUl) / 2;
  dPhi = (dUr - dUl) / WHEEL_SEPARATION;
  phi = phi + dPhi;
  x = x + dU * cos(phi);
  y = y + dU * sin(phi);

  //length += sprintf(buf + length, "%f, %f\n", x, y);
  //printf("%f, %f\n", x, y);
  //printf("%f, %f\tSpeed: %d, %d\n", x, y, *speedl->data, *speedr->data);

  /*printf("X: %f, Y: %f, Phi: %f\t", x, y, phi);
  printf("dUl: %f, dUr: %f, dU: %f, dPhi: %f\t", dUl, dUr, dU, dPhi);
  printf("Left: %d, Right: %d\n", p->left_delta_pos, p->right_delta_pos);*/
}

/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */
void reset_odo(odotype *p) {
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->left_enc_old = 0; // Since the motors are reset in main, these should be set to zero and not the current position
  p->left_delta_pos = p->right_delta_pos = 0;
  x = y = phi = 0;
}

void update_odo(odotype *p) {
  p->right_delta_pos = p->right_enc - p->right_enc_old;
  if (p->right_delta_pos > 0x8000) p->right_delta_pos -= 0x10000;
  else if (p->right_delta_pos < -0x8000) p->right_delta_pos += 0x10000;
  p->right_enc_old = p->right_enc;
  p->right_pos += p->right_delta_pos * p->cr;

  p->left_delta_pos = p->left_enc - p->left_enc_old;
  if (p->left_delta_pos > 0x8000) p->left_delta_pos -= 0x10000;
  else if (p->left_delta_pos < -0x8000) p->left_delta_pos += 0x10000;
  p->left_enc_old = p->left_enc;
  p->left_pos += p->left_delta_pos * p->cl;
}

void update_motcon(motiontype *p, linesensortype *line, irsensortype *ir) {
  if (p->cmd !=0) {
    p->finished=0;
    switch (p->cmd) {
      case mot_stop:
      	p->curcmd = mot_stop;
      	break;

      case mot_move:
      case mot_move_bwd:
      case mot_follow_black:
      case mot_follow_white:
      case mot_follow_wall_left:
        p->startpos = (p->left_pos + p->right_pos)/2.0;
        p->curcmd = p->cmd;
        break;

      case mot_turn:
        if (p->angle > 0) { // left
          //p->startpos = p->right_pos;
          p->startpos = p->right_pos - p->left_pos;
        } else { // Right
          //p->startpos = p->left_pos;
          p->startpos = p->left_pos - p->right_pos;
          //printf("Start pos: %f\n", p->startpos);
        }
        p->curcmd = mot_turn;
        break;
     }
     p->cmd=0;
   }

  switch (p->curcmd) {
    case mot_stop:
       p->motorspeed_l = 0;
       p->motorspeed_r = 0;
       break;

     case mot_move:
       if ((p->right_pos+p->left_pos)/2.0 - p->startpos >= p->dist) {
         p->finished = 1;
         p->motorspeed_l = p->motorspeed_r = 0;
       }
       else
        p->motorspeed_l = p->motorspeed_r = p->speedcmd;
      break;

     case mot_move_bwd:
       if ((p->right_pos+p->left_pos)/2.0 - p->startpos <= -p->dist) {
         p->finished = 1;
         p->motorspeed_l = p->motorspeed_r = 0;
       }
       else
        p->motorspeed_l = p->motorspeed_r = -p->speedcmd;
      break;

    case mot_follow_black:
    case mot_follow_white:
      if ((p->right_pos+p->left_pos)/2.0 - p->startpos >= p->dist) {
        p->finished = 1;
        p->motorspeed_l = p->motorspeed_r = 0;
      } else {
        double error;
        if (p->curcmd == mot_follow_black)
        	error = line->center_mass_neighbors[0] - 4.5;
        else
        	error = line->center_mass_neighbors[1] - 4.5;
        error *= p->speedcmd;
        double pSpeed = kPfollow * error;
        double dSpeed = kDfollow * (error - follow_error_old);
        double speed = pSpeed + dSpeed;
        follow_error_old = error;
        //printf("Speed: %f\n", speed);
        if (speed < 0) {
          p->motorspeed_l = p->speedcmd - speed;
          p->motorspeed_r = p->speedcmd + speed;
        } else {
          p->motorspeed_l = p->speedcmd - speed;
          p->motorspeed_r = p->speedcmd + speed;
        }
      }
      break;

    case mot_follow_wall_left:
      if ((p->right_pos+p->left_pos)/2.0 - p->startpos >= p->dist) {
        p->finished = 1;
        p->motorspeed_l = p->motorspeed_r = 0;
      } else {
        //printf("Dist: %f %f\n", p->wall_dist, ir->value[0]);
        if (ir->value[0] > p->wall_dist + 0.005) {
          p->motorspeed_l = p->speedcmd;
          p->motorspeed_r = p->speedcmd * 1.3;
        } else if (ir->value[0] < p->wall_dist - 0.005) {
          p->motorspeed_l = p->speedcmd * 1.3;
          p->motorspeed_r = p->speedcmd;
        } else {
          p->motorspeed_l = p->speedcmd;
          p->motorspeed_r = p->speedcmd;
        }
      }
      break;

#if 0
     case mot_turn:
       if (p->angle > 0) { // Left
          p->motorspeed_l = 0;
	  if (p->right_pos - p->startpos <= p->angle * p->w)
	      p->motorspeed_r = p->speedcmd;
	  else {
            p->motorspeed_r = 0;
            p->finished = 1;
	  }
	}
	else { // Right
          p->motorspeed_r = 0;
	  if (p->left_pos - p->startpos <= fabs(p->angle) * p->w)
	      p->motorspeed_l = p->speedcmd;
	  else {
            p->motorspeed_l = 0;
            p->finished = 1;
	  }
	}

     break;
#else
	case mot_turn:
	  //printf("Angle: %f\n", p->angle);
	  if (p->angle > 0) { // Left
      double pos = p->right_pos - p->left_pos - p->startpos;
      //printf("Left Pos: %f\n", pos);
      if (pos < (p->angle - TURN_ANGLE) * p->w) {
        p->motorspeed_r = p->speedcmd;
        p->motorspeed_l = -p->speedcmd;
      } else if (pos > (p->angle + TURN_ANGLE) * p->w) {
        p->motorspeed_r = -p->speedcmd;
        p->motorspeed_l = p->speedcmd;
      } else {
        p->motorspeed_r = p->motorspeed_l = 0;
        p->finished = 1;
      }
    } else { // Right
      double pos = p->left_pos - p->right_pos - p->startpos;
      //printf("Right pos: %f\n", pos);
      if (pos < (fabs(p->angle) - TURN_ANGLE) * p->w) {
        p->motorspeed_l = p->speedcmd;
        p->motorspeed_r = -p->speedcmd;
      } else if (pos > (fabs(p->angle) + TURN_ANGLE) * p->w) {
        p->motorspeed_l = -p->speedcmd;
        p->motorspeed_r = p->speedcmd;
      } else {
        p->motorspeed_l = p->motorspeed_r = 0;
  	    p->finished = 1;
      }
    }
	  break;
#endif
   }
}

int followBlackLine(motiontype *mot, double dist, double speed, int time) {
   if (time == 0) {
     mot->cmd = mot_follow_black;
     mot->speedcmd = speed;
     mot->dist = dist * DIST_CAL;
     mot->startAngle = phi; // Reset angle
     mot->x0 = x;
     mot->y0 = y;
     return 0;
   } else
     return mot->finished;
}

int followWhiteLine(motiontype *mot, double dist, double speed, int time) {
   if (time == 0) {
     mot->cmd = mot_follow_white;
     mot->speedcmd = speed;
     mot->dist = dist * DIST_CAL;
     mot->startAngle = phi; // Reset angle
     mot->x0 = x;
     mot->y0 = y;
     return 0;
   } else
     return mot->finished;
}

int followWallLeft(motiontype *mot, double dist, double speed, double wall_dist, int time) {
   if (time == 0) {
     mot->cmd = mot_follow_wall_left;
     mot->speedcmd = speed;
     mot->dist = dist * DIST_CAL;
     mot->startAngle = phi; // Reset angle
     mot->wall_dist = wall_dist;
     mot->x0 = x;
     mot->y0 = y;
     return 0;
   } else
     return mot->finished;
}

int fwd(motiontype *mot, double dist, double speed, int time) {
   if (time == 0) {
     mot->cmd = mot_move;
     mot->speedcmd = speed;
     mot->dist = dist * DIST_CAL;
     mot->startAngle = phi; // Reset angle
     mot->x0 = x;
     mot->y0 = y;
     return 0;
   } else
     return mot->finished;
}

int bwd(motiontype *mot, double dist, double speed, int time) {
   if (time == 0) {
     mot->cmd = mot_move_bwd;
     mot->speedcmd = speed;
     mot->dist = dist * DIST_CAL;
     mot->startAngle = phi; // Reset angle
     mot->x0 = x;
     mot->y0 = y;
     return 0;
   } else
     return mot->finished;
}

int turn(motiontype *mot, double angle, double speed, int time) {
   if (time == 0) {
     mot->cmd = mot_turn;
     mot->speedcmd = speed;
     mot->angle = angle;
     mot->startAngle = phi; // Reset angle
     return 0;
   }
   else
     return mot->finished;
}

void sm_update(smtype *p) {
  if (p->state != p->oldstate) {
    printState(p->state);
    p->time = 0;
    p->oldstate = p->state;
    follow_error_old = 0;
  } else
    p->time++;
}

void printState(int state) {
  switch (state) {
    case ms_init:
      printf("ms_init\n");
      break;
    case ms_fwd:
      printf("ms_fwd\n");
      break;
    case ms_fwd_time:
      printf("ms_fwd_time\n");
      break;
    case ms_bwd:
      printf("ms_bwd\n");
      break;
    case ms_fwd_fixed:
      printf("ms_fwd_fixed\n");
      break;
    case ms_fwd_cross_black:
      printf("ms_fwd_cross_black\n");
      break;
    case ms_fwd_cross_white:
      printf("ms_fwd_cross_white\n");
      break;
    case ms_fwd_ir_left:
      printf("ms_fwd_ir_left\n");
      break;
    case ms_fwd_ir_wall_left:
      printf("ms_fwd_ir_wall_left\n");
      break;
    case ms_fwd_ir_wall_right:
      printf("ms_fwd_ir_wall_right\n");
      break;
    case ms_fwd_gate_left:
      printf("ms_fwd_gate_left\n");
      break;
    case ms_fwd_gate_left_check:
      printf("ms_fwd_gate_left_check\n");
      break;
    case ms_follow_white:
      printf("ms_follow_white\n");
      break;
    case ms_follow_white_cross_black:
      printf("ms_follow_white_cross_black\n");
      break;
    case ms_follow_black:
      printf("ms_follow_black\n");
      break;
    case ms_follow_black_gate_left:
      printf("ms_follow_black_gate_left\n");
      break;
    case ms_follow_black_gate_left_check:
      printf("ms_follow_black_gate_left_check\n");
      break;
    case ms_follow_black_gate_left_right:
      printf("ms_follow_black_gate_left_right\n");
      break;
    case ms_follow_black_box:
      printf("ms_follow_black_box\n");
      break;
    case ms_follow_black_cross:
      printf("ms_follow_black_cross\n");
      break;
    case ms_follow_wall_left:
      printf("ms_follow_wall_left\n");
      break;
    case ms_turn_left:
      printf("ms_turn_left\n");
      break;
    case ms_turn_right:
      printf("ms_turn_right\n");
      break;
    case ms_turn_around:
      printf("ms_turn_around\n");
      break;
    case ms_center_line_black:
      printf("ms_center_line_black\n");
      break;
    case ms_center_line_black_left:
      printf("ms_center_line_black_left\n");
      break;
    case ms_center_line_black_right:
      printf("ms_center_line_black_right\n");
    case ms_center_angle_first:
      printf("ms_center_angle_first\n");
      break;
    case ms_ir_dist:
      printf("ms_ir_dist\n");
      break;
    case ms_gate_left:
      printf("ms_gate_left\n");
      break;
    case ms_gate_left_right:
      printf("ms_gate_left_right\n");
      break;
    case ms_center_angle:
      printf("ms_center_angle\n");
      break;
    case ms_reset_state:
      printf("ms_reset_state\n");
      break;
    case ms_wait_1s:
      printf("ms_wait_1s\n");
      break;
    case ms_stop:
      printf("ms_stop\n");
      break;
    case ms_end:
      printf("ms_end\n");
      break;
    default:
      printf("Unknown state\n");
      break;
  }
}
