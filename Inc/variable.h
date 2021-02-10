#ifndef __VARIABLE_H
#define __VARIABLE_H

#include <stdint.h>

//---------------------------------------------------------------------
// 列挙型
//---------------------------------------------------------------------
typedef enum {
  North = 0,
  West = 1,
  South = 2,
  East = 3,
}t_direction;

typedef enum {
  front = 0,
  left = 1,
  rear = 2,
  right = 3,
  pivo_rear = 4,
}t_act;

//---------------------------------------------------------------------
// 構造体定義
//---------------------------------------------------------------------
// trapezoid struct
typedef struct {
  float jerk;
  float accel_max;
  float max_velocity;
  float start_velocity;
  float end_velocity;
  float accel_t1;
  float accel_t2;
  float accel_t3;
  float down_t1;
  float down_t2;
  float down_t3;
  float const_t;
  int8_t run_flag;
  int8_t direction;
}t_trapezoid;

// run data
typedef struct {
  float accel;
  float velocity;
  float velocity_before;
  float distance;
}t_run;

// normal run data
typedef struct {
  float velocity;
  float accel;
}t_normal_param;

// division ( 偏差 )
typedef struct {
  float cumulative;
  float difference;
}t_deviation;

// sensor data
typedef struct {
  int16_t now;
  int16_t reference;      // 真ん中のときのセンサー値
  int16_t threshold;      // 閾値
  int16_t diff;     // 差分
  int16_t diff_1ms; // 1msec前
  uint8_t is_wall;        // 壁があるかどうか判断
  // 斜め閾値
}t_sensor;

// senosr data buff
typedef struct {
  int16_t before_1ms;
  int16_t before_2ms;
  int16_t before_3ms;
  int16_t before_4ms;
  int16_t before_5ms;
  int16_t now;
}t_sensosr_log;

// PID param
typedef struct {
  float kp;
  float ki;
  float kd;
}t_PID_param;

// motor duty
typedef struct {
  int32_t left;
  int32_t right;
}t_duty;

// slarom_offset
typedef struct {
  float in;
  float out;
}t_slarom_offset;

// slarom parameter
typedef struct {
  float angular_accel;
  float max_angular_velocity;
  t_slarom_offset left;
  t_slarom_offset right;
}t_slarom_parameter;

// enc value
typedef struct {
  int16_t left;
  int16_t right;
  float rpmm_l;
  float rpmm_r;
}t_enc_value;

typedef struct {
  uint32_t column[33];  // 縦壁　すなわち東壁
  uint32_t row[33];     // 横壁　すなわち北壁
  uint32_t column_known[33]; // 縦壁調べたかどうか 
  uint32_t row_known[33]; // 横壁調べたかどうか
  uint8_t save;
}t_walldata;

typedef struct {
  uint8_t x;
  uint8_t y;
  uint8_t direction;
}t_position;


//---------------------------------------------------------------------
// 構造体グローバル変数
//---------------------------------------------------------------------
// 速度、台形加速関連
extern t_run left_real;         // 左モーターの速度
extern t_run right_real;        // 右モーターの速度
extern t_run translation_real;
extern t_run rotation_real;     // 角加速度
extern t_run translation_ideal; // 理想の併進方向情報
extern t_run rotation_ideal;    // 理想の回転方向情報
extern t_trapezoid translation_trape_param; // 併進方向の台形加速情報
extern t_trapezoid rotation_trape_param;    // 回転方向の台形加速情報
extern t_normal_param run_param;          // 直線の台形加速の速度
extern t_normal_param rotation_param;     // 回転方向の台形加速の速度 
extern volatile uint8_t motion_end_flag;

// スラローム関連
extern t_slarom_parameter slarom300;  // 500 mm/sec のスラロームパラメータ

// PID関連
extern t_PID_param translation_gain;      // 速度ゲイン
extern t_PID_param rotation_gain;          // 回転ゲイン
extern t_PID_param sensor_gain;           // センサのゲイン
extern t_deviation run_left_deviation;    // 直線の差分
extern t_deviation run_right_deviation;   // 直線の差分
extern t_deviation rotation_deviation;     // 回転の差分

// duty , encoder 
extern t_enc_value enc_value;
extern t_duty duty;

// sensor data 
extern t_sensor sen_front; // 前壁
extern t_sensor sen_fl;
extern t_sensor sen_fr;
extern t_sensor sen_l; // 横左壁
extern t_sensor sen_r; // 横右壁

extern t_sensosr_log sen_fl_log;  // 前左壁
extern t_sensosr_log sen_fr_log;  // 前右壁

extern t_sensosr_log sen_l_log;  // 横左壁
extern t_sensosr_log sen_r_log;  // 横右壁

// maze
extern t_walldata wall_data;  // 壁情報
extern t_walldata wall_bit; // すべてに壁が入ってる壁情報

extern t_position mypos;      // 座標

//---------------------------------------------------------------------
// 変数
//---------------------------------------------------------------------
extern float batt_monitor;   // 電圧チェック

extern int8_t mode_counter;  // modeのカウンタ
extern float mode_distance;  // modeの選択用距離
extern volatile int32_t cnt_motion;   // motionカウンタ
extern volatile int32_t cnt_act;      // 動作の時間を保持するカウンタ

extern int8_t sidewall_control_flag;  // 横壁制御用フラグ
extern int8_t frontwall_control_flag;  // 前壁制御フラグ
extern int8_t wall_out_flag;           // 壁切れ用フラグ
extern int8_t dirwall_control_flag;    // 斜め壁制御用フラグ

extern uint8_t motion_queue[256]; // 最短の動作を入れておく変数
extern int16_t motion_last;     // motion last
extern int16_t motion_end;      // motion end

extern float accel_data;  // フェイルセーフ用の加速度の値を取得する変数

extern int8_t failSafe_flag; // フェイルセーフ用フラグ
extern volatile int8_t wait_flag;   // 待っているときかどうかのフラグ

extern uint32_t maze_step[32][32];   // 歩数マップ 

extern uint16_t enc_l;
extern uint16_t enc_r;

extern float distance_l;
extern float distance_r;

#endif /* __VARIABLE_H */