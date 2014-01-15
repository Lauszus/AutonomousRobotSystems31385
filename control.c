#include "control.h"

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
  for (i = 1; i < p->length; i++) {
    if (p->value[i] < p->lowest_val) {
      p->lowest_val = p->value[i];
      p->lowest_pos = i;
    }
  }

  if (p->lowest_pos > 0)
    p->lowest_val_right = p->value[p->lowest_pos - 1];
  else
    p->lowest_val_right = -1;
  if (p->lowest_pos < p->length - 1)
    p->lowest_val_left = p->value[p->lowest_pos + 1];
  else
    p->lowest_val_left = -1;

  for (i=0; i<= 7; i++) {
    if (p->value[i] > BLACK_LINE_FOUND_VALUE)
	break;
  }
  if (i == 8) {
    //printf("Black line found\n");
    p->black_line_found =1;
  }
  else
    p->black_line_found =0;

  double sumTop = 0, sumBot = 0;
  for (i = (p->lowest_pos > 0 ? p->lowest_pos - 1 : p->lowest_pos); i <= (p->lowest_pos < p->length - 1 ? p->lowest_pos + 1 : p->lowest_pos); i++) {
    double intensity = (p->lowest_val < 0.5 ? 1 - p->value[i] : p->value[i]);
    sumTop += (i + 1) * intensity;
    sumBot += intensity;
  }
  p->center_mass_neighbors = sumTop / sumBot;

  sumTop = sumBot = 0;
  for (i = 0; i < p->length; i++) {
    double intensity = (p->lowest_val < 0.5 ? 1 - p->value[i] : p->value[i]);
    sumTop += (i + 1) * intensity;
    sumBot += intensity;
  }
  p->center_mass = sumTop / sumBot;
}

void printLineSensor(linesensortype *p) {
  uint8_t i;
  printf("Values: ");
  for (i = 0; i < p->length; i++)
    printf("%f ", p->value[i]);
  printf(" Low val (l,c,r): %f,%f,%f Pos: %d CM: %f %f\n", p->lowest_val_left, p->lowest_val, p->lowest_val_right, p->lowest_pos, p->center_mass, p->center_mass_neighbors);
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
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
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

void update_motcon(motiontype *p, linesensortype *line) {
  if (p->cmd !=0) {
    p->finished=0;
    switch (p->cmd) {
      case mot_stop:
	p->curcmd = mot_stop;
	break;

      case mot_move:
      case mot_move_bwd:
      case mot_follow_black:
	p->startpos = (p->left_pos + p->right_pos)/2.0;
	p->curcmd = p->cmd;
	break;

      case mot_turn:
	if (p->angle > 0) // Left
	  //p->startpos = p->right_pos;
	  p->startpos = p->right_pos - p->left_pos;
	else // Right
	  //p->startpos = p->left_pos;
	  p->startpos = p->left_pos - p->right_pos;
	  //printf("Start pos: %f\n", p->startpos);
	p->curcmd = mot_turn;
        break;
     }
     p->cmd=0;
   }

   switch (p->curcmd) {
     case mot_stop:
       p->motorspeed_l=0;
       p->motorspeed_r=0;
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
       if ((p->right_pos+p->left_pos)/2.0 - p->startpos >= p->dist) {
          p->finished = 1;
	  p->motorspeed_l = p->motorspeed_r = 0;
       }
       else {
	 static double error_old = 0;
	 double error = line->center_mass_neighbors - 4.5;
	 error *= p->speedcmd;
	 double pSpeed = kPfollow * error;
	 double dSpeed = kDfollow * (error - error_old);
	 double speed = pSpeed + dSpeed;
	 error_old = error;
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
            if (pos <= p->angle * p->w) {
              p->motorspeed_r = p->speedcmd;
	      p->motorspeed_l = -p->speedcmd;
            } else {
              p->motorspeed_r = p->motorspeed_l = 0;
              p->finished = 1;
            }
          } else { // Right
	    double pos = p->left_pos - p->right_pos - p->startpos;
            //printf("Right pos: %f\n", pos);
            if (pos <= fabs(p->angle) * p->w) {
              p->motorspeed_l = p->speedcmd;
              p->motorspeed_r = -p->speedcmd;
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
    p->time = 0;
    p->oldstate = p->state;
  } else
    p->time++;
}
