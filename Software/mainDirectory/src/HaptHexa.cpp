#include <DynamixelWorkbench.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <ncurses.h>

DynamixelWorkbench dxl_wb;

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/joystick.h>
#include <string>
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <sys/time.h>

enum leg_num
{
  front_right,
  middle_right,
  rear_right,
  rear_left,
  middle_left,
  front_left,
};

class PID_parm
{
public:
  double DELTA_T;
  double KP = 2.50;
  double KD = 0.0001;
  double KI = 50.0;
  double int_err = 0.0;
  double dif[2];
  double currentTime;
  double pastTime;
};
class support_polygon
{
public:
  double long_diagonal = 44;
  double short_diagonal = 22 * sqrt(3.0);
  double side = 22;
};

enum mode
{
  auto_attitude_mode,
  manualMode,
};

class hexapod_body_state
{
public:
  const double side = 8.0, long_diagonal = 16.0, short_diagonal = 8 * sqrt(3.0);
  double yaw = 0.0, pitch = 0.0, roll = 0.0;
  double cog_height = 12.5;
  double ZMP_x, ZMP_y;
  int mode = manualMode;
  int mode_is_alrdy_changed;
};

class jy901_state
{
public:
  double yaw = 0.0;
  double roll = 0.0;
  double pich = 0.0;
  double accX;
  double accY;
  double accZ;
  double roll_offset = 0.0;
  double pitch_offset = 0.0;
  int is_calibration = 0;
};

class leg_state
{
public:
  const double coxa_length = 4, femur_length = 10, tibia_length = 12.5;
  int32_t coxa_encoder_val, femur_encoder_val, tibia_encoder_val; //value that order to encoder between 0 - 4048
  double coxa_arg, femur_arg, tibia_arg;                          //argument of joint -π[rad]〜+π[rad]
  double dx = 0.0, dy = 0.0, dz = 0.0;
  double x, y, z;
  double dx_home = 0.0, dy_home = 0.0, dz_home = 0.0;
  double x_home = 0.0, y_home = 0.0, z_home = 0.0;
  double x_base, y_base, z_base;
  double trajectory_hight = 10.0, trajectory_width = 5.0;
  double trajectory_yaw = 0.0, trajectory_pitch = 0.0;
  double trajectory_velocity;
};

typedef struct
{
  uint16_t X;
  uint16_t Y;
  uint16_t A;
  uint16_t B;
  uint16_t LB;
  uint16_t LT;
  uint16_t RB;
  uint16_t RT;
  uint16_t start;
  uint16_t back;
  int16_t axes1_x;
  int16_t axes1_y;
  int16_t axes0_x;
  int16_t axes0_y;
} controler_state;

void write_controler_state(controler_state *controler, js_event event)
{
  switch (event.type)
  {
  case JS_EVENT_BUTTON:
    if (event.number == 1)
    {
      controler->A = event.value;
    }
    if (event.number == 2)
    {
      controler->B = event.value;
    }
    if (event.number == 0)
    {
      controler->X = event.value;
    }
    if (event.number == 3)
    {
      controler->Y = event.value;
    }
    if (event.number == 4)
    {
      controler->LB = event.value;
    }
    if (event.number == 5)
    {
      controler->RB = event.value;
    }
    if (event.number == 6)
    {
      controler->LT = event.value;
    }
    if (event.number == 7)
    {
      controler->RT = event.value;
    }
    if (event.number == 9)
    {
      controler->start = event.value;
    }
    if (event.number == 0)
    {
      controler->X = event.value;
    }
    if (event.number == 3)
    {
      controler->Y = event.value;
    }
    if (event.number == 8)
    {
      controler->back = event.value;
      // printf("bock %d", controler->back);
    }

    break;
  case JS_EVENT_AXIS:
    if (event.number == 0)
    {
      controler->axes0_x = event.value;
    }
    if (event.number == 1)
    {
      controler->axes0_y = -event.value;
    }
    if (event.number == 2)
    {
      controler->axes1_x = event.value;
    }
    if (event.number == 3)
    {
      controler->axes1_y = -event.value;
    }
    break;
  default:
    /* Ignore init events. */
    break;
  }
}

void rotate_trajectory_depending_joy(leg_state *tmp_leg, hexapod_body_state *tmp_body_state, support_polygon *support_hexagon, controler_state *controler, double count)
{ //this is open loop controll as test

  int j = front_left;
  double delta_yaw;
  if (controler->LB || controler->RB)
  {
    if (controler->RB)
    {
      delta_yaw = (M_PI / 18.0) * sin(count * M_PI * 2 * 0.0020125);
    }
    if (controler->LB)
    {
      delta_yaw = -(M_PI / 18.0) * sin(count * M_PI * 2 * 0.0020125);
    }

    // int k = middle_right;
    for (int i = 0; i <= j; i++)
    {

      if (i == front_left)
      {
        tmp_leg[i].x = (0.5 * support_hexagon->long_diagonal * sin((M_PI / 3.0) + delta_yaw) - 0.5 * tmp_body_state->short_diagonal);
        tmp_leg[i].y = -(0.5 * support_hexagon->long_diagonal * cos((M_PI / 3.0) + delta_yaw) - 0.5 * tmp_body_state->side);
        tmp_leg[i].z = tmp_leg[i].z_home + 3 * cos(count * M_PI * 2 * 0.0020125);
        if (tmp_leg[i].z <= tmp_leg[i].z_home)
        {
          tmp_leg[i].z = tmp_leg[i].z_home;
        }
      }
      if (i == middle_right)
      {
        tmp_leg[i].x = 0.5 * support_hexagon->long_diagonal * cos(delta_yaw) - 0.5 * tmp_body_state->long_diagonal;
        tmp_leg[i].y = 0.5 * support_hexagon->long_diagonal * sin(delta_yaw);
        tmp_leg[i].z = tmp_leg[i].z_home + 3 * cos(count * M_PI * 2 * 0.0020125);
        if (tmp_leg[i].z <= tmp_leg[i].z_home)
        {
          tmp_leg[i].z = tmp_leg[i].z_home;
        }
      }
      if (i == rear_left)
      {
        tmp_leg[i].x = (0.5 * support_hexagon->long_diagonal * sin((M_PI / 3.0) - delta_yaw) - 0.5 * tmp_body_state->short_diagonal);
        tmp_leg[i].y = (0.5 * support_hexagon->long_diagonal * cos((M_PI / 3.0) - delta_yaw) - 0.5 * tmp_body_state->side);
        tmp_leg[i].z = tmp_leg[i].z_home + 3 * cos(count * M_PI * 2 * 0.0020125);
        if (tmp_leg[i].z <= tmp_leg[i].z_home)
        {
          tmp_leg[i].z = tmp_leg[i].z_home;
        }
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      if (i == rear_right)
      {
        tmp_leg[i].x = (0.5 * support_hexagon->long_diagonal * sin((M_PI / 3.0) - delta_yaw) - 0.5 * tmp_body_state->short_diagonal);
        tmp_leg[i].y = -(0.5 * support_hexagon->long_diagonal * cos((M_PI / 3.0) - delta_yaw) - 0.5 * tmp_body_state->side);
        tmp_leg[i].z = tmp_leg[i].z_home - 3 * cos(count * M_PI * 2 * 0.0020125);
        if (tmp_leg[i].z <= tmp_leg[i].z_home)
        {
          tmp_leg[i].z = tmp_leg[i].z_home;
        }
      }
      if (i == middle_left)
      {
        tmp_leg[i].x = 0.5 * support_hexagon->long_diagonal * cos(-delta_yaw) - 0.5 * tmp_body_state->long_diagonal;
        tmp_leg[i].y = 0.5 * support_hexagon->long_diagonal * sin(-delta_yaw);
        tmp_leg[i].z = tmp_leg[i].z_home - 3 * cos(count * M_PI * 2 * 0.0020125);
        if (tmp_leg[i].z <= tmp_leg[i].z_home)
        {
          tmp_leg[i].z = tmp_leg[i].z_home;
        }
      }
      if (i == front_right)
      {
        tmp_leg[i].x = (0.5 * support_hexagon->long_diagonal * sin((M_PI / 3.0) + delta_yaw) - 0.5 * tmp_body_state->short_diagonal);
        tmp_leg[i].y = (0.5 * support_hexagon->long_diagonal * cos((M_PI / 3.0) + delta_yaw) - 0.5 * tmp_body_state->side);
        tmp_leg[i].z = tmp_leg[i].z_home - 3 * cos(count * M_PI * 2 * 0.0020125);
        if (tmp_leg[i].z <= tmp_leg[i].z_home)
        {
          tmp_leg[i].z = tmp_leg[i].z_home;
        }
      }
    }
  }
}

void set_val_from_controller(leg_state *tmp_leg, js_event event, controler_state *controler, hexapod_body_state *body_state, support_polygon *support_hex)
{
  //コントローラクラスの値をロボットに適用
  double x0, x1;
  double y0, y1;
  int j = front_left;
  //x0,y0は左のジョイスティック（進行方向用）
  //x1,y1は右のジョイスティック(姿勢制御用)
  x0 = (double)controler->axes0_x;
  y0 = (double)controler->axes0_y;

  x1 = (double)controler->axes1_x;
  y1 = (double)controler->axes1_y;

  for (int i = 0; i <= j; i++)
  {
    tmp_leg[i].trajectory_yaw = -(atan2(y0, x0) - (M_PI / 2));
    //printf("trj yaw from joy %f \n", tmp_leg[i].trajectory_yaw);
  }
  for (int i = 0; i <= j; i++)
  {
    tmp_leg[i].trajectory_velocity = (sqrt(pow(abs(x0), 2.0) + pow(abs(y0), 2.0))) / 36000;
  }
  body_state->roll = -(1.5 * M_PI / 18.0) * (x1 / 36000);
  body_state->pitch = (1.5 * M_PI / 18.0) * (y1 / 36000);

  if (controler->A == 1)
  {
    body_state->cog_height = body_state->cog_height + 0.1;
  }
  if (controler->B == 1)
  {
    body_state->cog_height = body_state->cog_height - 0.1;
  }
  if (controler->X)
  {
    support_hex->long_diagonal = support_hex->long_diagonal + 0.1;
    support_hex->short_diagonal = support_hex->long_diagonal * 0.5 * sqrt(3.0);
  }
  if (controler->Y)
  {
    support_hex->long_diagonal = support_hex->long_diagonal - 0.1;
    support_hex->short_diagonal = support_hex->long_diagonal * 0.5 * sqrt(3.0);
  }
  // printf("x:%f y:%f \n", x1, y1);
}

void set_val_from_jy901_and_controller(leg_state *tmp_leg, controler_state *controler, jy901_state *jy901, hexapod_body_state *body_state, PID_parm *PIDpitch, PID_parm *PIDroll)
{
  //コントローラクラスの値をロボットに適用
  double x0, x1;
  double y0, y1;
  int j = front_left;
  double delta_roll;
  double delta_pitch;
  struct timeval currentTime;

  //x0,y0は左のジョイスティック（進行方向用）
  //x1,y1は右のジョイスティック（重心制御）
  x0 = (double)controler->axes0_x;
  y0 = (double)controler->axes0_y;

  x1 = (double)controler->axes1_x;
  y1 = (double)controler->axes1_y;

  for (int i = 0; i <= j; i++)
  {
    tmp_leg[i].trajectory_yaw = -(atan2(y0, x0) - (M_PI / 2));
    //printf("trj yaw from joy %f \n", tmp_leg[i].trajectory_yaw);
  }
  for (int i = 0; i <= j; i++)
  {
    tmp_leg[i].trajectory_velocity = (sqrt(pow(abs(x0), 2.0) + pow(abs(y0), 2.0))) / 36000;
  }

  double p, i, d;
  gettimeofday(&currentTime, NULL);

  //微分のための微小時間の計算
  PIDroll->pastTime = PIDroll->currentTime;
  PIDroll->currentTime = (currentTime.tv_usec) * 0.000001;
  PIDroll->DELTA_T = PIDroll->currentTime - PIDroll->pastTime;
  PIDpitch->DELTA_T = PIDroll->DELTA_T;

  //ロール軸のPID制御
  //偏差の計算
  PIDroll->dif[0] = PIDroll->dif[1];
  PIDroll->dif[1] = 0.0 - jy901->roll;
  //偏差の積分
  //PIDroll->int_err = PIDroll->int_err + PIDroll->dif[1] * PIDroll->DELTA_T;
  PIDroll->int_err = PIDroll->int_err + PIDroll->dif[1] * 0.01;

  p = PIDroll->KP * PIDroll->dif[1];
 // d = PIDroll->KD * (PIDroll->dif[0] - PIDroll->dif[1]) / PIDroll->DELTA_T;
  d = PIDroll->KD * (PIDroll->dif[0] - PIDroll->dif[1]) / 0.01;
  i = PIDroll->KI * PIDroll->int_err;

  // body_state->roll = body_state->roll + p - d;//謎制御
  body_state->roll = p + i - d; //pid制御
  //body_state->roll = p; //p制御

  //ピッチ軸のPD制御
  PIDpitch->dif[0] = PIDpitch->dif[1];
  PIDpitch->dif[1] = 0.0 + jy901->pich;
  //偏差の積分
 // PIDpitch->int_err = PIDpitch->int_err + PIDpitch->dif[1] * PIDroll->DELTA_T;
 PIDpitch->int_err = PIDpitch->int_err + PIDpitch->dif[1] * 0.01;

  p = PIDpitch->KP * PIDpitch->dif[1];
  //d = PIDpitch->KD * (PIDpitch->dif[0] - PIDpitch->dif[1]) / PIDpitch->DELTA_T;
  d = PIDpitch->KD * (PIDpitch->dif[0] - PIDpitch->dif[1]) / 0.01;
  i = PIDpitch->KI * PIDpitch->int_err;

  //body_state->pitch = body_state->pitch * 2 + p - d;//謎制御
  body_state->pitch = p + i - d; //pi制御
  printf("%f",d);
  //body_state->pitch = p; //p制御

  body_state->ZMP_y = 5.0 * (y1 / 36000.0);
  body_state->ZMP_x = 5.0 * (x1 / 36000.0);
  if (controler->A == 1)
  {
    body_state->cog_height = body_state->cog_height + 0.1;
  }
  if (controler->B == 1)
  {
    body_state->cog_height = body_state->cog_height - 0.1;
  }
  // if (controler->Y == 1)
  // {
  //   for (int i = 0; i <= j; i++)
  //   {
  //     tmp_leg[i].trajectory_hight = tmp_leg[i].trajectory_hight + 0.05;
  //   }
  // }
  // if (controler->X == 1)
  // {
  //   for (int i = 0; i <= j; i++)
  //   {
  //     tmp_leg[i].trajectory_hight = tmp_leg[i].trajectory_hight - 0.05;
  //   }
  // }
}

void flat_terrain_walk_rajectory(leg_state *tmp_leg, hexapod_body_state *tmp_body_state, double count)
{
  int j = front_left;
  double position_arg;
  double norm;
  double yaw_x, yaw_y, yaw_z;

  // int k = middle_right;
  for (int i = 0; i <= j; i++)
  {
    tmp_leg[i].x = tmp_leg[i].x_home;
    tmp_leg[i].y = tmp_leg[i].y_home;
    tmp_leg[i].z = tmp_leg[i].z_home;
    if (i == middle_right)
    {
      //軌道の進行方向(trajectory_yaw)とボディーのピッチ角とロール角から，軌道をどれだけ傾けるか（trajectroy_pich）を計算している。
      tmp_leg[i].trajectory_pitch = atan(sin(tmp_leg[i].trajectory_yaw) * tan(-tmp_body_state->roll) + cos(tmp_leg[i].trajectory_yaw) * tan(tmp_body_state->pitch));
      // printf("trajectory pich :%f trajecotyr yaw :%f \n ", tmp_leg[i].trajectory_pitch, tmp_leg[i].trajectory_yaw);

      tmp_leg[i].x = tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI) - tmp_leg[i].trajectory_hight * sin(tmp_leg[i].trajectory_yaw) * sin(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125 + M_PI);
      tmp_leg[i].y = -(tmp_leg[i].trajectory_width * cos(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI) - tmp_leg[i].trajectory_hight * cos(tmp_leg[i].trajectory_yaw) * sin(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125 + M_PI));
      tmp_leg[i].z = tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI) + tmp_leg[i].trajectory_hight * cos(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125 + M_PI);
      if (tmp_leg[i].z <= tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI))
      {
        tmp_leg[i].x = tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI);
        tmp_leg[i].y = -(tmp_leg[i].trajectory_width * cos(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI));
        tmp_leg[i].z = tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI);
      }
    }

    if (i == middle_left)
    {
      tmp_leg[i].trajectory_pitch = atan(sin(tmp_leg[i].trajectory_yaw) * tan(-tmp_body_state->roll) + cos(tmp_leg[i].trajectory_yaw) * tan(tmp_body_state->pitch));

      tmp_leg[i].x = -(tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125) - tmp_leg[i].trajectory_hight * sin(tmp_leg[i].trajectory_yaw) * sin(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125));
      tmp_leg[i].y = (tmp_leg[i].trajectory_width * cos(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125) - tmp_leg[i].trajectory_hight * cos(tmp_leg[i].trajectory_yaw) * sin(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125));
      tmp_leg[i].z = tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125) + tmp_leg[i].trajectory_hight * cos(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125);
      if (tmp_leg[i].z <= tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125))
      {
        tmp_leg[i].x = -(tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125));
        tmp_leg[i].y = (tmp_leg[i].trajectory_width * cos(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125));
        tmp_leg[i].z = tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125);
      }
    }
    if (i == front_right)
    {
      tmp_leg[i].trajectory_pitch = atan(sin(tmp_leg[i].trajectory_yaw) * tan(-tmp_body_state->roll) + cos(tmp_leg[i].trajectory_yaw) * tan(tmp_body_state->pitch));

      tmp_leg[i].y = (tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125) - tmp_leg[i].trajectory_hight * sin(tmp_leg[i].trajectory_yaw) * sin(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125));
      tmp_leg[i].x = (tmp_leg[i].trajectory_width * cos(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125) - tmp_leg[i].trajectory_hight * cos(tmp_leg[i].trajectory_yaw) * sin(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125));
      tmp_leg[i].z = tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125) + tmp_leg[i].trajectory_hight * cos(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125);
      if (tmp_leg[i].z <= tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125))
      {
        tmp_leg[i].y = (tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125));
        tmp_leg[i].x = (tmp_leg[i].trajectory_width * cos(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125));
        tmp_leg[i].z = tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125);
      }
    }
    if (i == front_left)
    {
      tmp_leg[i].trajectory_pitch = atan(sin(tmp_leg[i].trajectory_yaw) * tan(-tmp_body_state->roll) + cos(tmp_leg[i].trajectory_yaw) * tan(tmp_body_state->pitch));

      tmp_leg[i].y = (tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI) - tmp_leg[i].trajectory_hight * sin(tmp_leg[i].trajectory_yaw) * sin(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125 + M_PI));
      tmp_leg[i].x = (tmp_leg[i].trajectory_width * cos(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI) - tmp_leg[i].trajectory_hight * cos(tmp_leg[i].trajectory_yaw) * sin(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125 + M_PI));
      tmp_leg[i].z = tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI) + tmp_leg[i].trajectory_hight * cos(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125 + M_PI);
      if (tmp_leg[i].z <= tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI))
      {
        tmp_leg[i].y = (tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI));
        tmp_leg[i].x = (tmp_leg[i].trajectory_width * cos(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI));
        tmp_leg[i].z = tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI);
      }
    }
    if (i == rear_left)
    {
      tmp_leg[i].trajectory_pitch = atan(sin(tmp_leg[i].trajectory_yaw) * tan(-tmp_body_state->roll) + cos(tmp_leg[i].trajectory_yaw) * tan(tmp_body_state->pitch));

      tmp_leg[i].y = -(tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI) - tmp_leg[i].trajectory_hight * sin(tmp_leg[i].trajectory_yaw) * sin(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125 + M_PI));
      tmp_leg[i].x = -(tmp_leg[i].trajectory_width * cos(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI) - tmp_leg[i].trajectory_hight * cos(tmp_leg[i].trajectory_yaw) * sin(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125 + M_PI));
      tmp_leg[i].z = tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI) + tmp_leg[i].trajectory_hight * cos(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125 + M_PI);
      if (tmp_leg[i].z <= tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI))
      {
        tmp_leg[i].y = -(tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI));
        tmp_leg[i].x = -(tmp_leg[i].trajectory_width * cos(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI));
        tmp_leg[i].z = tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125 + M_PI);
      }
    }
    if (i == rear_right)
    {
      tmp_leg[i].trajectory_pitch = atan(sin(tmp_leg[i].trajectory_yaw) * tan(-tmp_body_state->roll) + cos(tmp_leg[i].trajectory_yaw) * tan(tmp_body_state->pitch));

      tmp_leg[i].y = -(tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125) - tmp_leg[i].trajectory_hight * sin(tmp_leg[i].trajectory_yaw) * sin(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125));
      tmp_leg[i].x = -(tmp_leg[i].trajectory_width * cos(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125) - tmp_leg[i].trajectory_hight * cos(tmp_leg[i].trajectory_yaw) * sin(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125));
      tmp_leg[i].z = tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125) + tmp_leg[i].trajectory_hight * cos(tmp_leg[i].trajectory_pitch) * cos(count * M_PI * 2 * 0.0020125);
      if (tmp_leg[i].z <= tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125))
      {
        tmp_leg[i].y = -(tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125));
        tmp_leg[i].x = -(tmp_leg[i].trajectory_width * cos(tmp_leg[i].trajectory_yaw) * cos(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125));
        tmp_leg[i].z = tmp_leg[i].trajectory_width * sin(tmp_leg[i].trajectory_pitch) * sin(count * M_PI * 2 * 0.0020125);
      }
    }
  }
  for (int i = 0; i <= j; i++)
  {
    tmp_leg[i].y = tmp_leg[i].trajectory_velocity * tmp_leg[i].y;
    tmp_leg[i].x = tmp_leg[i].trajectory_velocity * tmp_leg[i].trajectory_velocity * tmp_leg[i].x;
    tmp_leg[i].z = tmp_leg[i].trajectory_velocity * tmp_leg[i].z;
  }
  for (int i = 0; i <= j; i++)
  {
    tmp_leg[i].y = tmp_leg[i].trajectory_velocity * tmp_leg[i].y + tmp_leg[i].y_home;
    tmp_leg[i].x = tmp_leg[i].trajectory_velocity * tmp_leg[i].trajectory_velocity * tmp_leg[i].x + tmp_leg[i].x_home;
    tmp_leg[i].z = tmp_leg[i].trajectory_velocity * tmp_leg[i].z + tmp_leg[i].z_home;
  }
}

class controller_state
{
public:
  uint16_t X;
  uint16_t Y;
  uint16_t A;
  uint16_t B;
  uint16_t LB;
  uint16_t RB;
  uint16_t start;
  int16_t axes1_x;
  int16_t axes1_y;
  int16_t axes0_x;
  int16_t axes0_y;
};

void set_initialvalue_to_leg(leg_state leg[], hexapod_body_state *body_state, support_polygon *support_hexagon)
{
  leg[front_right].x_base = 0.5 * (support_hexagon->short_diagonal - body_state->short_diagonal);
  leg[front_right].y_base = 0.5 * (support_hexagon->side - body_state->side);
  leg[front_right].z_base = -body_state->cog_height;

  leg[front_left].x_base = 0.5 * (support_hexagon->short_diagonal - body_state->short_diagonal);
  leg[front_left].y_base = -0.5 * (support_hexagon->side - body_state->side);
  leg[front_left].z_base = -body_state->cog_height;

  leg[middle_left].x_base = 0.5 * (support_hexagon->long_diagonal - body_state->long_diagonal);
  leg[middle_left].y_base = 0;
  leg[middle_left].z_base = -body_state->cog_height;

  leg[middle_right].x_base = 0.5 * (support_hexagon->long_diagonal - body_state->long_diagonal);
  leg[middle_right].y_base = 0;
  leg[middle_right].z_base = -body_state->cog_height;

  leg[rear_right].x_base = 0.5 * (support_hexagon->short_diagonal - body_state->short_diagonal);
  leg[rear_right].y_base = -0.5 * (support_hexagon->side - body_state->side);
  leg[rear_right].z_base = -body_state->cog_height;

  leg[rear_left].x_base = 0.5 * (support_hexagon->short_diagonal - body_state->short_diagonal);
  leg[rear_left].y_base = 0.5 * (support_hexagon->side - body_state->side);
  leg[rear_left].z_base = -body_state->cog_height;
}

void set_joint_arg_by_inv_dynamics(leg_state tmp_leg[])
{
  double tmp_x, tmp_y, tmp_z;
  int j = front_left;
  for (int i = 0; i <= j; i++)
  {
    tmp_leg[i].coxa_arg = atan2(tmp_leg[i].y, tmp_leg[i].x);

    tmp_x = tmp_leg[i].x * cos(tmp_leg[i].coxa_arg) + tmp_leg[i].y * sin(tmp_leg[i].coxa_arg) - tmp_leg[i].coxa_length;
    tmp_y = -tmp_leg[i].x * sin(tmp_leg[i].coxa_arg) + tmp_leg[i].y * cos(tmp_leg[i].coxa_arg);
    tmp_z = tmp_leg[i].z;

    tmp_leg[i].tibia_arg = -(M_PI - acos((pow(tmp_leg[i].femur_length, 2) + pow(tmp_leg[i].tibia_length, 2) - pow(tmp_x, 2) - pow(tmp_z, 2)) / (2 * tmp_leg[i].femur_length * tmp_leg[i].tibia_length)));
    tmp_leg[i].femur_arg = atan2(tmp_z, tmp_x) + acos((pow(tmp_leg[i].femur_length, 2) - pow(tmp_leg[i].tibia_length, 2) + pow(tmp_x, 2) + pow(tmp_z, 2)) / (2 * tmp_leg[i].femur_length * sqrt(pow(tmp_x, 2) + pow(tmp_z, 2))));
  }
}

void controll_attitude_by_yaw_pich(leg_state leg[], hexapod_body_state *body_state, support_polygon *support_hexagon, double count)
{
  set_initialvalue_to_leg(leg, body_state, support_hexagon);
  //ピッチ角から前足のホームポジションの差分を計算
  leg[front_right].dx_home = 0.5 * (support_hexagon->short_diagonal * cos(body_state->pitch) - body_state->short_diagonal) - body_state->ZMP_y - leg[front_right].x_base;
  leg[front_right].dy_home = 0.5 * (support_hexagon->side - body_state->side) - leg[front_right].y_base;
  leg[front_right].dz_home = -(body_state->cog_height - 0.5 * support_hexagon->short_diagonal * sin(body_state->pitch)) - leg[front_right].z_base;

  leg[front_left].dx_home = 0.5 * (support_hexagon->short_diagonal * cos(body_state->pitch) - body_state->short_diagonal) - body_state->ZMP_y - leg[front_left].x_base;
  leg[front_left].dy_home = -0.5 * (support_hexagon->side - body_state->side) - leg[front_left].y_base;
  leg[front_left].dz_home = -(body_state->cog_height - 0.5 * support_hexagon->short_diagonal * sin(body_state->pitch)) - leg[front_left].z_base;

  //ピッチ軸から中脚のホームポジションの差分の計算
  leg[middle_left].dx_home = 0.5 * (support_hexagon->long_diagonal - body_state->long_diagonal) - leg[middle_left].x_base;
  leg[middle_left].dy_home = -body_state->ZMP_y - leg[middle_left].y_base;
  leg[middle_left].dz_home = -body_state->cog_height - leg[middle_left].z_base;

  leg[middle_right].dx_home = 0.5 * (support_hexagon->long_diagonal - body_state->long_diagonal) - leg[middle_right].x_base;
  leg[middle_right].dy_home = body_state->ZMP_y - leg[middle_right].y_base;
  leg[middle_right].dz_home = -body_state->cog_height - leg[middle_right].z_base;

  //ピッチ軸から後脚のホームポジションの差分を計算
  leg[rear_right].dx_home = 0.5 * (support_hexagon->short_diagonal * cos(body_state->pitch) - body_state->short_diagonal) + body_state->ZMP_y - leg[rear_right].x_base;
  leg[rear_right].dy_home = -0.5 * (support_hexagon->side - body_state->side) - leg[rear_right].y_base;
  leg[rear_right].dz_home = -(body_state->cog_height + 0.5 * support_hexagon->short_diagonal * sin(body_state->pitch)) - leg[rear_right].z_base;

  leg[rear_left].dx_home = 0.5 * (support_hexagon->short_diagonal * cos(body_state->pitch) - body_state->short_diagonal) + body_state->ZMP_y - leg[rear_left].x_base;
  leg[rear_left].dy_home = 0.5 * (support_hexagon->side - body_state->side) - leg[rear_left].y_base;
  leg[rear_left].dz_home = -(body_state->cog_height + 0.5 * support_hexagon->short_diagonal * sin(body_state->pitch)) - leg[rear_left].z_base;

  ////////////////////////////////////////////////////ここからロール角に基づく制御
  //ロール軸から前脚のホームポジションの差分を計算
  leg[front_right].dx_home = leg[front_right].dx_home + 0.5 * (support_hexagon->short_diagonal - body_state->short_diagonal) - leg[front_right].x_base;
  leg[front_right].dy_home = leg[front_right].dy_home + 0.5 * (support_hexagon->side * cos(body_state->roll) - body_state->side) - body_state->ZMP_x - leg[front_right].y_base;
  leg[front_right].dz_home = leg[front_right].dz_home - (body_state->cog_height + 0.5 * support_hexagon->side * sin(body_state->roll)) - leg[front_right].z_base;

  leg[front_left].dx_home = leg[front_left].dx_home + 0.5 * (support_hexagon->short_diagonal - body_state->short_diagonal) - leg[front_left].x_base;
  leg[front_left].dy_home = leg[front_left].dy_home - (0.5 * (support_hexagon->side * cos(body_state->roll) - body_state->side) + body_state->ZMP_x) - leg[front_left].y_base;
  leg[front_left].dz_home = leg[front_left].dz_home - (body_state->cog_height - 0.5 * support_hexagon->side * sin(body_state->roll)) - leg[front_left].z_base;

  //ロール軸から中脚のホームポジションの差分を計算
  leg[middle_left].dx_home = leg[middle_left].dx_home + 0.5 * (support_hexagon->long_diagonal * cos(body_state->roll) - body_state->long_diagonal) + body_state->ZMP_x - leg[middle_left].x_base;
  leg[middle_left].dy_home = leg[middle_left].dy_home + 0 - leg[middle_left].y_base;
  leg[middle_left].dz_home = leg[middle_left].dz_home - (body_state->cog_height - 0.5 * (support_hexagon->long_diagonal * sin(body_state->roll))) - leg[middle_left].z_base;

  leg[middle_right].dx_home = leg[middle_right].dx_home + 0.5 * (support_hexagon->long_diagonal * cos(body_state->roll) - body_state->long_diagonal) - body_state->ZMP_x - leg[middle_right].x_base;
  leg[middle_right].dy_home = leg[middle_right].dy_home + 0 - leg[middle_right].y_base;
  leg[middle_right].dz_home = leg[middle_right].dz_home - (body_state->cog_height + 0.5 * (support_hexagon->long_diagonal * sin(body_state->roll))) - leg[middle_right].z_base;

  //ロール軸から後脚のホームポジションの差分を計算
  leg[rear_right].dx_home = leg[rear_right].dx_home + 0.5 * (support_hexagon->short_diagonal - body_state->short_diagonal) - leg[rear_right].x_base;
  leg[rear_right].dy_home = leg[rear_right].dy_home - (0.5 * (support_hexagon->side * cos(body_state->roll) - body_state->side) - body_state->ZMP_x) - leg[rear_right].y_base;
  leg[rear_right].dz_home = leg[rear_right].dz_home - (body_state->cog_height + 0.5 * support_hexagon->side * sin(body_state->roll)) - leg[rear_right].z_base;

  leg[rear_left].dx_home = leg[rear_left].dx_home + 0.5 * (support_hexagon->short_diagonal - body_state->short_diagonal) - leg[rear_left].x_base;
  leg[rear_left].dy_home = leg[rear_left].dy_home + 0.5 * (support_hexagon->side * cos(body_state->roll) - body_state->side) + body_state->ZMP_x - leg[rear_left].y_base;
  leg[rear_left].dz_home = leg[rear_left].dz_home - (body_state->cog_height - 0.5 * support_hexagon->side * sin(body_state->roll)) - leg[rear_left].z_base;

  // printf("leg[front_right]: dx = %f , dy = %f,dz = %f\n", leg[front_right].dx_home, leg[front_right].dy_home, leg[front_right].dz_home);
  // printf("leg[rear_left]: dx = %f , dy = %f,dz = %f\n", leg[rear_left].dx_home, leg[rear_left].dy_home, leg[rear_left].dz_home);
  int j = front_left;
  for (int i = 0; i <= j; i++)
  {
    leg[i].x_home = leg[i].x_base + leg[i].dx_home;
    leg[i].y_home = leg[i].y_base + leg[i].dy_home;
    leg[i].z_home = leg[i].z_base + leg[i].dz_home;
  };
}

void pub_encoder_val_to_all_dyanmixel(leg_state leg[], const char *log)
{
  bool result;
  //radから0から4096の分解能による指令値に変更
  //モータの正負の違いもここて調節し直す
  leg[front_right].femur_encoder_val = -1 * (leg[front_right].femur_arg) * (2048 / M_PI) + 2048;
  leg[front_right].tibia_encoder_val = leg[front_right].tibia_arg * (2048 / M_PI) + 2048;
  leg[front_right].coxa_encoder_val = leg[front_right].coxa_arg * (2048 / M_PI) + 2048;

  // printf("coxa = %f, femur = %f, tibia = %f \n ", leg[front_left]->coxa_arg * 180 / M_PI, leg[front_left]->femur_arg * 180 / M_PI, leg[front_left]->tibia_arg * 180 / M_PI);
  leg[front_left].coxa_encoder_val = leg[front_left].coxa_arg * (2048 / M_PI) + 2048;
  leg[front_left].femur_encoder_val = leg[front_left].femur_arg * (2048 / M_PI) + 2048;
  leg[front_left].tibia_encoder_val = -1 * (leg[front_left].tibia_arg) * (2048 / M_PI) + 2048;

  leg[middle_right].femur_encoder_val = -1 * (leg[middle_right].femur_arg) * (2048 / M_PI) + 2048;
  leg[middle_right].tibia_encoder_val = leg[middle_right].tibia_arg * (2048 / M_PI) + 2048;
  leg[middle_right].coxa_encoder_val = leg[middle_right].coxa_arg * (2048 / M_PI) + 2048;

  leg[middle_left].femur_encoder_val = (leg[middle_left].femur_arg) * (2048 / M_PI) + 2048;
  leg[middle_left].tibia_encoder_val = -1 * (leg[middle_left].tibia_arg) * (2048 / M_PI) + 2048;
  leg[middle_left].coxa_encoder_val = leg[middle_left].coxa_arg * (2048 / M_PI) + 2048;

  leg[rear_right].femur_encoder_val = -1 * (leg[rear_right].femur_arg) * (2048 / M_PI) + 2048;
  leg[rear_right].tibia_encoder_val = leg[rear_right].tibia_arg * (2048 / M_PI) + 2048;
  leg[rear_right].coxa_encoder_val = leg[rear_right].coxa_arg * (2048 / M_PI) + 2048;

  leg[rear_left].femur_encoder_val = leg[rear_left].femur_arg * (2048 / M_PI) + 2048;
  leg[rear_left].tibia_encoder_val = -1 * (leg[rear_left].tibia_arg) * (2048 / M_PI) + 2048;
  leg[rear_left].coxa_encoder_val = leg[rear_left].coxa_arg * (2048 / M_PI) + 2048;

  //それぞれの関節に指令値を代入
  //デバッグの時はreusultの中身を見る
  result = dxl_wb.addBulkWriteParam(1, "Goal_Position", leg[front_right].coxa_encoder_val, &log);
  result = dxl_wb.addBulkWriteParam(2, "Goal_Position", leg[front_right].femur_encoder_val, &log);
  result = dxl_wb.addBulkWriteParam(3, "Goal_Position", leg[front_right].tibia_encoder_val, &log);
  //////////////////////////////////////////////////////////////////////////////
  result = dxl_wb.addBulkWriteParam(4, "Goal_Position", leg[middle_right].coxa_encoder_val, &log);
  result = dxl_wb.addBulkWriteParam(5, "Goal_Position", leg[middle_right].femur_encoder_val, &log);
  result = dxl_wb.addBulkWriteParam(6, "Goal_Position", leg[middle_right].tibia_encoder_val, &log);
  //////////////////////////////////////////////////////////////////////////////
  result = dxl_wb.addBulkWriteParam(7, "Goal_Position", leg[rear_right].coxa_encoder_val, &log);
  result = dxl_wb.addBulkWriteParam(8, "Goal_Position", leg[rear_right].femur_encoder_val, &log);
  result = dxl_wb.addBulkWriteParam(9, "Goal_Position", leg[rear_right].tibia_encoder_val, &log);
  /////////////////////////////////////////////////////////////////////////////
  result = dxl_wb.addBulkWriteParam(10, "Goal_Position", leg[rear_left].coxa_encoder_val, &log);
  result = dxl_wb.addBulkWriteParam(11, "Goal_Position", leg[rear_left].femur_encoder_val, &log);
  result = dxl_wb.addBulkWriteParam(12, "Goal_Position", leg[rear_left].tibia_encoder_val, &log);
  //////////////////////////////////////////////////////////////////////////////
  result = dxl_wb.addBulkWriteParam(13, "Goal_Position", leg[middle_left].coxa_encoder_val, &log);
  result = dxl_wb.addBulkWriteParam(14, "Goal_Position", leg[middle_left].femur_encoder_val, &log);
  result = dxl_wb.addBulkWriteParam(15, "Goal_Position", leg[middle_left].tibia_encoder_val, &log);
  ///////////////////////////////////////////////////////////////////////////////
  result = dxl_wb.addBulkWriteParam(16, "Goal_Position", leg[front_left].coxa_encoder_val, &log);
  result = dxl_wb.addBulkWriteParam(17, "Goal_Position", leg[front_left].femur_encoder_val, &log);
  result = dxl_wb.addBulkWriteParam(18, "Goal_Position", leg[front_left].tibia_encoder_val, &log);
  dxl_wb.bulkWrite(&log);
}

void read_jy(int fd, jy901_state *jy901)
{
  //printf("///////////////////////////////////////////////starting_reading_jy//////////////////////////\n");
  unsigned char read_buf[1];
  unsigned char jy_buf[11];
  int i = 0;
  int j = 0;

  memset(&read_buf, '\0', sizeof(read_buf));
  memset(&jy_buf, '\0', sizeof(jy_buf));
  int num_bytes;

  while (1)
  {
    //printf("staring_jy_reading_loop %d \n", i);
    num_bytes = read(fd, &read_buf, sizeof(read_buf));
    if (num_bytes < 0)
    {
      printf("Error reading: %s \n", strerror(errno));
    }
    //printf("%x \n", read_buf[0]);
    if (read_buf[0] == 0x55)
    {
      printf("\n");
      i = 0;
      jy_buf[i] = read_buf[0];
    }

    jy_buf[i] = read_buf[0];

    if (i == 10)
    {
      // printf("|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|%x|\n", jy_buf[0], jy_buf[1], jy_buf[2], jy_buf[3], jy_buf[4], jy_buf[5], jy_buf[6], jy_buf[7], jy_buf[8], jy_buf[9], jy_buf[10]);
      if (jy_buf[1] == 0x53)
      {
        jy901->pich = (((read_buf[6] << 8) | read_buf[5]) / 182.54);
        jy901->roll = (((read_buf[4] << 8) | read_buf[3]) / 182.54);
        if (180.0 <= jy901->pich && jy901->pich <= 360.0)
        {
          jy901->pich = jy901->pich - 360.0;
        }
        if (180.0 <= jy901->roll && jy901->roll <= 360.0)
        {
          jy901->roll = jy901->roll - 360.0;
        }
        jy901->pich = jy901->pich * (M_PI / 180.0);
        jy901->roll = jy901->roll * (M_PI / 180.0);
      }
      jy901->pich = jy901->pich - jy901->pitch_offset;
      jy901->roll = jy901->roll - jy901->roll_offset;
      break;
    }
    i++;
  }
}

int main(int argc, char *argv[])
{
  ///////////////////////コントローラ関係/////////////////////
  int fd = open("/dev/input/js0", O_NONBLOCK); //初期化
  struct js_event event;
  ////////////////////////dynamixedl関係/////////////////////
  const char *port_name = "/dev/ttyUSB0";
  int baud_rate = 2 * 1000 * 1000;
  uint16_t model_number = 0;
  uint8_t dxl_id[18];

  dxl_id[0] = 1;
  dxl_id[1] = 2;
  dxl_id[2] = 3;
  dxl_id[3] = 4;
  dxl_id[4] = 5;
  dxl_id[5] = 6;
  dxl_id[6] = 7;
  dxl_id[7] = 8;
  dxl_id[8] = 9;
  dxl_id[9] = 10;
  dxl_id[10] = 11;
  dxl_id[11] = 12;
  dxl_id[12] = 13;
  dxl_id[13] = 14;
  dxl_id[14] = 15;
  dxl_id[15] = 16;
  dxl_id[16] = 17;
  dxl_id[17] = 18;

  bool result = false;
  port_name = "/dev/ttyUSB0";
  const char *log;

  result = dxl_wb.init(port_name, baud_rate, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to init\n");
    return 0;
  }
  else
    printf("Succeed to init(%d)\n", baud_rate);

  //pingを使って通信を確認
  for (int cnt = 0; cnt <= 17; cnt++)
  {
    result = dxl_wb.ping(dxl_id[cnt], &model_number, &log);
    if (result == false)
    {
      printf("%s\n", log);
      printf("Failed to ping\n");
    }
    else
    {
      printf("Succeeded to ping\n");
      printf("id : %d, model_number : %d\n", dxl_id[cnt], model_number);
    }
    result = dxl_wb.jointMode(dxl_id[cnt], 0, 0, &log);
    if (result == false)
    {
      printf("%s\n", log);
      printf("Failed to change joint mode\n");
    }
    else
    {
      printf("Succeed to change joint mode\n");
    }
  }

  result = dxl_wb.initBulkWrite(&log);
  if (result == false)
  {
    printf("%s\n", log);
  }
  else
  {
    printf("%s\n", log);
  }

  result = dxl_wb.initBulkRead(&log);
  if (result == false)
  {
    printf("%s\n", log);
  }
  else
  {
    printf("%s\n", log);
  }

  result = dxl_wb.addBulkReadParam(dxl_id[0], "Present_Position", &log);

  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to add bulk read position param\n");
  }
  else
  {
    printf("%s\n", log);
  }
  ///////////////////ロボットの要素//////////////////////////
  double count = 0.0;
  controler_state *controler;
  PID_parm PIDpitch[1];
  PID_parm PIDroll[1];
  controler = (controler_state *)malloc(sizeof(controler_state));
  hexapod_body_state body_state[1];
  support_polygon support_hexagon[1];
  leg_state leg[6];
  jy901_state jy901[1];
  jy901_state terrainJy[1];

  //////////////////ロボット搭載のjy901関係の初期設定//////////////////////
  int serial_port = open("/dev/ttyUSB1", O_RDWR);
  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;
  memset(&tty, 0, sizeof tty);

  // Read in existing settings, and handle any error

  if (tcgetattr(serial_port, &tty) != 0)
  {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag |= CS8;            // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;                                                        // Disable echo
  tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
  tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
  tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 115200
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
  {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  //////////////////実験フィールド状態記録用のjy901関係の初期設定//////////////////////
  int tr_serial_port = open("/dev/ttyUSB2", O_RDWR);
  // Create new termios struc, we call it 'tty' for convention
  struct termios tr_tty;
  memset(&tty, 0, sizeof tr_tty);

  // Read in existing settings, and handle any error

  if (tcgetattr(serial_port, &tr_tty) != 0)
  {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tr_tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
  tr_tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
  tr_tty.c_cflag |= CS8;            // 8 bits per byte (most common)
  tr_tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
  tr_tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tr_tty.c_lflag &= ~ICANON;
  tr_tty.c_lflag &= ~ECHO;                                                        // Disable echo
  tr_tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
  tr_tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
  tr_tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
  tr_tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
  tr_tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

  tr_tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tr_tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tr_tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tr_tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 115200
  cfsetispeed(&tr_tty, B115200);
  cfsetospeed(&tr_tty, B115200);

  if (tcsetattr(tr_serial_port, TCSANOW, &tr_tty) != 0)
  {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
  ///////////////////実験データ記録ファイルのための準備//////////////////////////////////////////
  FILE *jyRecFile;
  jyRecFile = fopen("jyRec.txt", "w");
  fprintf(jyRecFile, "Kp= %f,Kd =%f \n", PIDpitch->KP, PIDpitch->KD);
  fprintf(jyRecFile, "stride = %f, trjHeight = %f,body_heigut %f\n", leg[front_left].trajectory_width * 2, leg[front_left].trajectory_hight, body_state->cog_height);
  fprintf(jyRecFile, "POSIXtime,terrainPitch,terrainroll,jyPitch,jyRoll,estimateTerrainPich,estimateTerrainRoll\n");
  struct timeval currentTime; //現在時刻を格納する構造体変数

  //メインのループ，拡張したいときはここを中心にいじる
  while (controler->start == 0)
  {

    //コントローラからの値の読み込み
    read(fd, &event, sizeof(event));
    write_controler_state(controler, event);
    if (controler->back == 1 && body_state->mode == auto_attitude_mode && body_state->mode_is_alrdy_changed == 0)
    {
      body_state->mode = manualMode;
      body_state->mode_is_alrdy_changed = 1;
    }

    if (controler->back == 1 && body_state->mode == manualMode && body_state->mode_is_alrdy_changed == 0)
    {
      body_state->mode = auto_attitude_mode;
      body_state->mode_is_alrdy_changed = 1;
    }

    if (controler->back == 0)
    {
      body_state->mode_is_alrdy_changed = 0;
    }

    //jy901のキャリブレーション
    if (controler->LT == 1 && jy901->is_calibration == 0)
    {
      jy901->roll_offset = jy901->roll;
      jy901->pitch_offset = jy901->pich;
      jy901->is_calibration = 1;
      // terrainJy->pitch_offset = terrainJy->pich;
      // terrainJy->roll_offset = terrainJy->roll;
    }
    if (controler->LT == 1 && terrainJy->is_calibration == 0)
    {
      terrainJy->pitch_offset = terrainJy->pich;
      terrainJy->roll_offset = terrainJy->roll;
      terrainJy->is_calibration = 1;
    }
    //jy901からの値の読み込み
    read_jy(serial_port, jy901); //ロボット搭載のjy901
    //read_jy(tr_serial_port, terrainJy); //実験フィールド用のjy901

    if (body_state->mode == manualMode)
    {
      //ボディーにコントローラからの情報を書き込む，（ヨー，ピッチなど）
      set_val_from_controller(leg, event, controler, body_state, support_hexagon);
    }
    if (body_state->mode == auto_attitude_mode)
    {
      PIDpitch->int_err =0.0;
      PIDroll->int_err = 0.0;
      //自動姿勢制御モードコントローラ+jy901からデータを取り代入する
      set_val_from_jy901_and_controller(leg, controler, jy901, body_state, PIDpitch, PIDroll);
    }

    controll_attitude_by_yaw_pich(leg, body_state, support_hexagon, count);
    flat_terrain_walk_rajectory(leg, body_state, count);
    rotate_trajectory_depending_joy(leg, body_state, support_hexagon, controler, count);

    //現在時刻の所得
    gettimeofday(&currentTime, NULL);
    // printf("/////////////hex Infoation//////////////////\n");
    printf("pich by Jy901: %f °\n", (jy901->pich / M_PI) * 180.0);
    printf("Roll by jy901: %f °\n", (jy901->roll / M_PI) * 180.0);
    // printf("pich by terrain: %f °\n", (terrainJy->pich / M_PI) * 180.0);
    // printf("Roll by terrain: %f °\n", (terrainJy->roll / M_PI) * 180.0);

    // printf("currentTime: %ld.%06lu \n", currentTime.tv_sec, currentTime.tv_usec);
   fprintf(jyRecFile, "%ld.%06lu,%f,%f,%f,%f,%f,%f,\n", currentTime.tv_sec, currentTime.tv_usec, (terrainJy->pich / M_PI) * 180.0, (terrainJy->roll / M_PI) * 180.0, (jy901->pich / M_PI) * 180.0, (jy901->roll / M_PI) * 180.0, (body_state->pitch / M_PI) * 180.0, (body_state->roll / M_PI) * 180.0);

    ////実験データを見やすくするためのマーカー
    if (controler->RT)
    {
      fprintf(jyRecFile, "////marker////\n");
    }
    set_joint_arg_by_inv_dynamics(leg);
    pub_encoder_val_to_all_dyanmixel(leg, log);
    count++;
    usleep(5000);
  }
  fclose(jyRecFile);
  close(serial_port);
  return 0;
}