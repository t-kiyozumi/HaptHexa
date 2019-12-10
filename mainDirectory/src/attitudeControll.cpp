#include <DynamixelWorkbench.h>
#include <stdio.h>
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

enum leg_num
{
  front_right,
  middle_right,
  rear_right,
  rear_left,
  middle_left,
  front_left,
};

class support_polygon
{
public:
  double long_diagonal = 44;
  double short_diagonal = 22 * sqrt(3.0);
  double side = 22;
};

class hexapod_body_state
{
public:
  const double side = 8.0, long_diagonal = 16.0, short_diagonal = 8 * sqrt(3.0);
  double yaw = 0.0, pitch = 0.0, roll = 0.0;
  double cog_height = 15.0;
  double ZMP_x = 0.0, ZMP_y;
};

class leg_state
{
public:
  const double coxa_length = 4, femur_length = 10, tibia_length = 15;
  int32_t coxa_encoder_val, femur_encoder_val, tibia_encoder_val; //value that order to encoder between 0 - 4048
  double coxa_arg, femur_arg, tibia_arg;                          //argument of joint -π[rad]〜+π[rad]
  double x, y, z;
  double x_home, y_home, z_home;
};

void flat_terrain_walk_rajectory(leg_state *tmp_leg, hexapod_body_state *tmp_body_state, double count)
{
  int j = front_left;
  for (int i = 0; i <= j; i++)
  {
    tmp_leg[i].x = tmp_leg[i].x_home;
    tmp_leg[i].y = tmp_leg[i].y_home;
    tmp_leg[i].z = tmp_leg[i].z_home;
  };
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
  };
}

void calc_and_assign_homeposition_from_pitch(leg_state leg[], hexapod_body_state *body_state, support_polygon *support_hexagon)
{
}

void calc_and_assign_homeposition_from_roll(leg_state leg[], hexapod_body_state *body_state, support_polygon *support_hexagon)
{
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  leg[front_right].x_home = 0.5 * leg[front_right].x_home;
  leg[front_right].x_home = 0.5 * leg[front_right].y_home;
  leg[front_right].z_home = 0.5 * leg[front_right].z_home;

  leg[front_left].x_home = 0.5 * leg[front_left].x_home;
  leg[front_left].y_home = 0.5 * leg[front_left].y_home;
  leg[front_left].z_home = 0.5 * leg[front_left].z_home;

  //中脚のホームポジションの決定
  leg[middle_left].x_home = 0.5 * leg[middle_left].x_home;
  leg[middle_left].y_home = 0.5 * leg[middle_left].y_home;
  leg[middle_left].z_home = 0.5 * leg[middle_left].z_home;

  leg[middle_right].x_home = 0.5 * leg[middle_right].x_home;
  leg[middle_right].y_home = 0.5 * leg[middle_right].y_home;
  leg[middle_right].z_home = 0.5 * leg[middle_right].z_home;

  //後脚のホームポジションの決定
  leg[rear_right].x_home = 0.5 * leg[rear_right].x_home;
  leg[rear_right].y_home = 0.5 * leg[rear_right].y_home;
  leg[rear_right].z_home = 0.5 * leg[rear_right].z_home;

  leg[rear_left].x_home = 0.5 * leg[rear_left].x_home;
  leg[rear_left].y_home = 0.5 * leg[rear_left].y_home;
  leg[rear_left].z_home = 0.5 * leg[rear_left].z_home;
}

void controll_attitude_by_yaw_pich(leg_state leg[], hexapod_body_state *body_state, support_polygon *support_hexagon, double count)
{
  ///ピッチ軸から前脚のホームポジションの決定
  leg[front_right].x_home = 0.5 * (support_hexagon->short_diagonal * cos(body_state->pitch) - body_state->short_diagonal);
  leg[front_right].y_home = 0.5 * (support_hexagon->short_diagonal * cos(body_state->pitch) - body_state->short_diagonal) * tan((1.0 / 6.0) * M_PI);
  leg[front_right].z_home = -(body_state->cog_height - 0.5 * support_hexagon->short_diagonal * sin(body_state->pitch));

  leg[front_left].x_home = 0.5 * (support_hexagon->short_diagonal * cos(body_state->pitch) - body_state->short_diagonal);
  leg[front_left].y_home = -0.5 * (support_hexagon->short_diagonal * cos(body_state->pitch) - body_state->short_diagonal) * tan((1.0 / 6.0) * M_PI);
  leg[front_left].z_home = -(body_state->cog_height - 0.5 * support_hexagon->short_diagonal * sin(body_state->pitch));

  //ピッチ軸から中脚のホームポジションの決定
  leg[middle_left].x_home = 0.5 * (support_hexagon->long_diagonal - body_state->long_diagonal);
  leg[middle_left].y_home = 0;
  leg[middle_left].z_home = -body_state->cog_height;

  leg[middle_right].x_home = 0.5 * (support_hexagon->long_diagonal - body_state->long_diagonal);
  leg[middle_right].y_home = 0;
  leg[middle_right].z_home = -body_state->cog_height;

  //ピッチ軸から後脚のホームポジションの決定
  leg[rear_right].x_home = 0.5 * (support_hexagon->short_diagonal * cos(body_state->pitch) - body_state->short_diagonal);
  leg[rear_right].y_home = -0.5 * (support_hexagon->short_diagonal * cos(body_state->pitch) - body_state->short_diagonal) * tan((1.0 / 6.0) * M_PI);
  leg[rear_right].z_home = -(body_state->cog_height + 0.5 * support_hexagon->short_diagonal * sin(body_state->pitch));

  leg[rear_left].x_home = 0.5 * (support_hexagon->short_diagonal * cos(body_state->pitch) - body_state->short_diagonal);
  leg[rear_left].y_home = 0.5 * (support_hexagon->short_diagonal * cos(body_state->pitch) - body_state->short_diagonal) * tan((1.0 / 6.0) * M_PI);
  leg[rear_left].z_home = -(body_state->cog_height + 0.5 * support_hexagon->short_diagonal * sin(body_state->pitch));

  ///ロール軸から前脚のホームポジションの決定
  leg[front_right].x_home = (1.0 / tan((1.0 / 6.0) * M_PI)) * 0.5 * (support_hexagon->side * cos(body_state->roll) - body_state->side) + leg[front_right].x_home;
  leg[front_right].y_home = 0.5 * (support_hexagon->side * cos(body_state->roll) - body_state->side) + leg[front_right].y_home;
  leg[front_right].z_home = -(body_state->cog_height + 0.5 * support_hexagon->side * sin(body_state->roll)) + leg[front_right].z_home;

  leg[front_left].x_home = (1.0 / tan((1.0 / 6.0) * M_PI)) * 0.5 * (support_hexagon->side * cos(body_state->roll) - body_state->side) + leg[front_left].x_home;
  leg[front_left].y_home = -0.5 * (support_hexagon->side * cos(body_state->roll) - body_state->side) + leg[front_left].y_home;
  leg[front_left].z_home = -(body_state->cog_height - 0.5 * support_hexagon->side * sin(body_state->roll)) + leg[front_left].z_home;


  //ロール軸から中脚のホームポジションの決定
  leg[middle_left].x_home = 0.5 * (support_hexagon->long_diagonal * cos(body_state->roll) - body_state->long_diagonal) + leg[middle_left].x_home;
  leg[middle_left].y_home = 0 + leg[middle_left].y_home;
  leg[middle_left].z_home = -(body_state->cog_height - 0.5 * (support_hexagon->long_diagonal * sin(body_state->roll))) + leg[middle_left].z_home;

  leg[middle_right].x_home = 0.5 * (support_hexagon->long_diagonal * cos(body_state->roll) - body_state->long_diagonal) + leg[middle_right].x_home;
  leg[middle_right].y_home = 0 + leg[middle_right].y_home;
  leg[middle_right].z_home = -(body_state->cog_height + 0.5 * (support_hexagon->long_diagonal * sin(body_state->roll))) + leg[middle_right].z_home;

  //ロール軸から後脚のホームポジションの決定
  leg[rear_right].x_home = (1.0 / tan((1.0 / 6.0) * M_PI)) * 0.5 * (support_hexagon->side * cos(body_state->roll) - body_state->side) + leg[rear_right].x_home;
  leg[rear_right].y_home = -0.5 * (support_hexagon->side * cos(body_state->roll) - body_state->side) + leg[rear_right].y_home;
  leg[rear_right].z_home = -(body_state->cog_height + 0.5 * support_hexagon->side * sin(body_state->roll)) + leg[rear_right].z_home;

  leg[rear_left].x_home = (1.0 / tan((1.0 / 6.0) * M_PI)) * 0.5 * (support_hexagon->side * cos(body_state->roll) - body_state->side) + leg[rear_left].x_home;
  leg[rear_left].y_home = 0.5 * (support_hexagon->side * cos(body_state->roll) - body_state->side) + leg[rear_left].y_home;
  leg[rear_left].z_home = -(body_state->cog_height - 0.5 * support_hexagon->side * sin(body_state->roll)) + leg[rear_left].z_home;

  //ベクトル合成を行ったのでこれを半分にする
  leg[front_right].x_home = 0.5 * leg[front_right].x_home;
  leg[front_right].y_home = 0.5 * leg[front_right].y_home;
  leg[front_right].z_home = 0.5 * leg[front_right].z_home;

  leg[front_left].x_home = 0.5 * leg[front_left].x_home;
  leg[front_left].y_home = 0.5 * leg[front_left].y_home;
  leg[front_left].z_home = 0.5 * leg[front_left].z_home;

  leg[middle_left].x_home = 0.5 * leg[middle_left].x_home;
  leg[middle_left].y_home = 0.5 * leg[middle_left].y_home;
  leg[middle_left].z_home = 0.5 * leg[middle_left].z_home;

  leg[middle_right].x_home = 0.5 * leg[middle_right].x_home;
  leg[middle_right].y_home = 0.5 * leg[middle_right].y_home;
  leg[middle_right].z_home = 0.5 * leg[middle_right].z_home;

  leg[rear_right].x_home = 0.5 * leg[rear_right].x_home;
  leg[rear_right].y_home = 0.5 * leg[rear_right].y_home;
  leg[rear_right].z_home = 0.5 * leg[rear_right].z_home;

  leg[rear_left].x_home = 0.5 * leg[rear_left].x_home;
  leg[rear_left].y_home = 0.5 * leg[rear_left].y_home;
  leg[rear_left].z_home = 0.5 * leg[rear_left].z_home;
}

void pub_encoder_val_to_all_dyanmixel(leg_state leg[], const char *log)
{
  leg[middle_left].coxa_arg = leg[middle_left].coxa_arg + M_PI / 5; //ロリコンが狂ったので公正
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

  double count = 0.0;
  hexapod_body_state body_state[1];
  support_polygon support_hexagon[1];
  leg_state leg[6];
  printf("sup_hex_long_diag %f\n", support_hexagon->long_diagonal);
  printf("fuch you");
  while (1)
  {
    body_state->roll = (1 / 18.0) * M_PI * sin(count * M_PI * 2 * 0.003);
    body_state->pitch = (3 / 18.0) * M_PI * cos(count * M_PI * 2 * 0.003);
    controll_attitude_by_yaw_pich(leg, body_state, support_hexagon, count);
    flat_terrain_walk_rajectory(leg, body_state, count);
    // printf("leg[middle_right]_arg: coxa = %f π,femur = %f　π,tibia = %f π\n", leg[middle_right].coxa_arg / M_PI, leg[middle_right].femur_arg / M_PI, leg[middle_right].tibia_arg / M_PI);
    // printf("leg[middle_right]_cordinate: x = %f , y = %f,z = %f\n", leg[middle_right].x, leg[middle_right].y, leg[middle_right].z);

    // printf("leg[front_right]_arg: coxa = %f π,femur = %f　π,tibia = %f π\n", leg[front_right].coxa_arg / M_PI, leg[front_right].femur_arg / M_PI, leg[front_right].tibia_arg / M_PI);
    printf("leg[front_right]_cordinate: x = %f , y = %f,z = %f\n", leg[front_right].x, leg[front_right].y, leg[front_right].z);
    printf("leg[front_right]_cordinate: x = %f , y = %f,z = %f\n", leg[front_right].x, leg[front_right].y, leg[front_right].z);

    // printf("leg[front_left]_arg: coxa = %f π,femur = %f　π,tibia = %f π\n", leg[front_left].coxa_arg / M_PI, leg[front_left].femur_arg / M_PI, leg[front_left].tibia_arg / M_PI);
    // printf("leg[front_left]_cordinate: x = %f , y = %f,z = %f\n", leg[front_left].x, leg[front_left].y, leg[front_left].z);

    set_joint_arg_by_inv_dynamics(leg);
    pub_encoder_val_to_all_dyanmixel(leg, log);
    count++;
    usleep(20000);
  }
  return 0;
}
