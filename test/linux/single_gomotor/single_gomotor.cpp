/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include "ethercat.h"
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <linux/types.h>
#include <linux/module.h>
#include "set.h"
#include "get.h"

#include "config.h"
#include "shm.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>

#define EC_TIMEOUTMON 500
#define NUM 1000
#define TH 2.0
// #define TH2 0.16666666667
#define TH2 1.0
#define MOTOR_NUM 1

#define USE_ACCELELERATION_TARGET_FLAG 1
#if USE_ACCELELERATION_TARGET_FLAG
double last_acc_set_time_ctrl_clock[MOTOR_NUM] = {0.0};
double acc_set_time_soem_clock[MOTOR_NUM] = {0.0};
double pos_at_acc_set_time[MOTOR_NUM] = {0.0};
double vel_at_acc_set_time[MOTOR_NUM] = {0.0};
double recent_pos_ref_only_used_by_accref[MOTOR_NUM] = {0.0};
double recent_vel_ref_only_used_by_accref[MOTOR_NUM] = {0.0};
#endif

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
boolean forceByteAlignment = FALSE;

typedef struct Cat_data {
    uint8 send[16];
    uint8* recv;
} cat_data;

cat_data motor[MOTOR_NUM];

static volatile int keepRunning = 1;
void signal_handler(int signum) {
  std::cerr<<std::endl<<std::endl<<std::endl<<std::endl<<"catch signum = "<<signum<<std::endl;
    keepRunning = 0;
}
int monitoring_print_cursor = 20;
std::vector<double> single_gomotor_sensor_shared(num_data_single_gomotor_sensor, 0.0);
std::vector<double> single_gomotor_command_shared(num_data_single_gomotor_command, 0.0);

/*送信関数*/
void set_output(uint16 slave_no, uint8 module_index, uint8* value)
{
    set_crc(value);  // CRCの値を計算
    uint8* data_ptr;

    data_ptr = ec_slave[slave_no].outputs;
    data_ptr += module_index * 16;
    for (int i = 0; i < 16; i++) {
        *data_ptr++ = *value++;
    }
}

/*
    初期化関数
    使用するモータによりidの値を変える
    idはRS485で使用するid
*/
void set_init()
{
    /*RS485通信で使うidの変更*/
    set_id(0, motor[0].send);
    //set_id(1, motor[1].send);
    //set_id(0, motor[2].send);

    /*指令値をすべて0に設定*/
    for (int i = 0; i < MOTOR_NUM; i++) {
        set_mode(1, motor[i].send);
        set_torque(0.00, motor[i].send);
        set_speed(0, motor[i].send);
        set_K_P(0, motor[i].send);
        set_K_W(0, motor[i].send);
        set_position(0, motor[i].send);
    }
}

// ロギング用のバッファを定義．
// 位置，速度の観測値と，目標位置，目標速度の指令値を保存する．
// 観測値は，C++プロセス内のタイムスタンプと一緒に保存する．
// 指令値は，書き込まれた時刻のタイムスタンプ，C++プロセス内のタイムスタンプと一緒に保存する．
// 指令値の計算に使用した加速度目標値，ゼロ次ホールド積分の初期位置と速度も保存する．
// 1ステップあたりのデータ数
const int obs_log_data_per_step = 3;
const int cmd_log_data_per_step = 7;
// ロギング用のバッファサイズ
const int log_buffer_size = 30000;
// ロギング用のバッファ
double obs_log_buffer[log_buffer_size][obs_log_data_per_step];
double cmd_log_buffer[log_buffer_size][cmd_log_data_per_step];
// ロギング用のバッファのインデックス
int obs_log_index = 0;
int cmd_log_index = 0;
// ロギング用のファイル名
std::string obs_log_filename = "single_gomotor_obs_log.csv";
std::string cmd_log_filename = "single_gomotor_cmd_log.csv";
// ロギング用の関数
void log_obs_data(double cpp_time, double position, double velocity)
{
    if (obs_log_index < log_buffer_size) {
        obs_log_buffer[obs_log_index][0] = cpp_time;
        obs_log_buffer[obs_log_index][1] = position;
        obs_log_buffer[obs_log_index][2] = velocity;
        obs_log_index++;
    } else {
        obs_log_buffer[0][0] = cpp_time;
        obs_log_buffer[0][1] = position;
        obs_log_buffer[0][2] = velocity;
        obs_log_index = 1;
    }
}
void log_cmd_data(double cpp_time, double python_set_time, double acc_cmd, double position_0, double velocity_0, double position_cmd, double velocity_cmd)
{
    if (cmd_log_index < log_buffer_size) {
        cmd_log_buffer[cmd_log_index][0] = cpp_time;
        cmd_log_buffer[cmd_log_index][1] = python_set_time;
        cmd_log_buffer[cmd_log_index][2] = acc_cmd;
        cmd_log_buffer[cmd_log_index][3] = position_0;
        cmd_log_buffer[cmd_log_index][4] = velocity_0;
        cmd_log_buffer[cmd_log_index][5] = position_cmd;
        cmd_log_buffer[cmd_log_index][6] = velocity_cmd;
        cmd_log_index++;
    } else {
        cmd_log_buffer[0][0] = cpp_time;
        cmd_log_buffer[0][1] = python_set_time;
        cmd_log_buffer[0][2] = acc_cmd;
        cmd_log_buffer[0][3] = position_0;
        cmd_log_buffer[0][4] = velocity_0;
        cmd_log_buffer[0][5] = position_cmd;
        cmd_log_buffer[0][6] = velocity_cmd;
        cmd_log_index = 1;
    }
}
void save_log_to_file()
{
    // ロギング用のファイルストリーム
    std::ofstream obs_log_file(obs_log_filename);
    std::ofstream cmd_log_file(cmd_log_filename);
    // ロギングの精度を設定
    obs_log_file.setf(std::ios::fixed);
    obs_log_file.precision(10);

    cmd_log_file.setf(std::ios::fixed);
    cmd_log_file.precision(10);

    // 観測データのログをファイルに保存

    for (int i = 0; i < obs_log_index; i++) {
        obs_log_file << obs_log_buffer[i][0] << ","
                     << obs_log_buffer[i][1] << ","
                     << obs_log_buffer[i][2] << "\n";
    }
    // 指令データのログをファイルに保存
    for (int i = 0; i < cmd_log_index; i++) {
        cmd_log_file << cmd_log_buffer[i][0] << ","
                     << cmd_log_buffer[i][1] << ","
                     << cmd_log_buffer[i][2] << ","
                     << cmd_log_buffer[i][3] << ","
                     << cmd_log_buffer[i][4] << ","
                     << cmd_log_buffer[i][5] << ","
                     << cmd_log_buffer[i][6] << "\n";
    }
    // ファイルを閉じる
    obs_log_file.close();
    cmd_log_file.close();
}

/*
    計測用関数
*/
void mesure(double* dat, double* ave, double* var, double* max, int* overcnt, int* overcnt2, double* min)
{
    double sum = 0;
    *max = 0;
    *min = 1000;
    *overcnt = 0;
    *overcnt2 = 0;
    for (int i = 0; i < NUM; i++) {
        if (*max < dat[i]) {
            *max = dat[i];
        }
        if (*min > dat[i]) {
            *min = dat[i];
        }
        if (dat[i] >= TH) {
            *overcnt += 1;
        }
        if (dat[i] >= TH2) {
            *overcnt2 += 1;
        }
        sum += dat[i];
    }
    *ave = sum / NUM;
    sum = 0;
    for (int i = 0; i < NUM; i++) {
        double j = dat[i] - *ave;
        sum += j * j;
    }
    *var = sum / NUM;
}

int get_time_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_nsec/1000;
}

/*
    ethercat通信するスレッド
*/
void simpletest(char* ifname)
{
    int i, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

    // clock_t st_clock[MOTOR_NUM] = {0}, end_clock[MOTOR_NUM] = {0};
    uint8 check[MOTOR_NUM];
    bool recv_fin[MOTOR_NUM];
    for (int i = 0; i < MOTOR_NUM; i++) {
        check[i] = 255;
        recv_fin[i] = TRUE;
    }
    double time_count[MOTOR_NUM][NUM];
    int time_index[MOTOR_NUM] = {0};
    int now_time[MOTOR_NUM] = {0};
    double max_time[MOTOR_NUM] = {0};
    double min_time[MOTOR_NUM] = {0};
    double ave_time[MOTOR_NUM] = {0};
    double var_time[MOTOR_NUM] = {0};
    int over_num[MOTOR_NUM] = {0};
    int over_num2[MOTOR_NUM] = {0};

    // 時間の詳細な計測
    int cycle_num = 0;
    int buf_size = 20000;
    uint8 check_buf[2][buf_size][MOTOR_NUM] = {}; // check_order, check
    int time_buf[3][buf_size][MOTOR_NUM] = {}; // start_time, end_time, elapsed_time
    int cycle_start_us;
    int cycle_end_us;

    // 正弦波信号
    double acc_amplitude = 1.5;
    double acc_frequency = 5.0; // 振動数
    double control_start_clock;

    printf("\033[2J\033[1;1H");  // 画面クリア
    printf("Starting single gomotor\n");

    int is_host = 1;
    ProcComm *proc_comm_sensor;
    ProcComm *proc_comm_command;
    proc_comm_sensor = new ProcComm(filename_data_single_gomotor_sensor, id_data_single_gomotor_sensor, num_data_single_gomotor_sensor, is_host);
    proc_comm_command = new ProcComm(filename_data_single_gomotor_command, id_data_single_gomotor_command, num_data_single_gomotor_command, is_host);

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname)) {
        printf("ec_init on %s succeeded.\n", ifname);
        /* find and auto-config slaves */


        if (ec_config_init(FALSE) > 0) {
            printf("%d slaves found and configured.\n", ec_slavecount);

            if (forceByteAlignment) {
                ec_config_map_aligned(&IOmap);
            } else {
                ec_config_map(&IOmap);
            }

            ec_configdc();

            printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0))
                oloop = 1;
            if (oloop > 64)
                oloop = 64;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1;
            if (iloop > 64)
                iloop = 64;

            printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            printf("in%d out%d", oloop, iloop);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 200;
            /* wait for all slaves to reach OP state */
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL)){
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            };
            if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
                // printf("\nfdsjfakl\n");
                // printf("\a");
                printf("Operational state reached for all SLAVEs.\n");
                // printf("\a");
                //printf("\033[10;1H");
                //printf("motor INFO");
                //printf("\033[%d;1H", MOTOR_NUM + 13);
                printf("data transmission speed");

                inOP = TRUE;
                set_init();

                for (uint i = 0; i < MOTOR_NUM; i++) {
                    motor[i].recv = get_recv_pointer(i);
                }
                clock_t cyc_f = 0, cyc_f_pre = 0;
                float tor = 0;
                float tor2 = 0;
                struct timespec t_st[MOTOR_NUM], t_end[MOTOR_NUM];
                /* cyclic loop */
                for (;;) {
                    cycle_start_us = get_time_us();
                    cyc_f = clock();
                    //printf("\033[%d;1H", 30);
                    double elapsedtime = (double)(cyc_f - cyc_f_pre) / CLOCKS_PER_SEC;
                    single_gomotor_command_shared = proc_comm_command->read_stdvec();
                    for (int i = 0; i < MOTOR_NUM; i++) {
		                if (recv_fin[i]) {
			                recv_fin[i] = FALSE;
                            motor[i].send[15] = check[i];
                            double torque_control = single_gomotor_command_shared[TORQUE_CMD_IDX*MOTOR_NUM + i];
                            double target_pos = single_gomotor_command_shared[POSITION_TARGET_IDX*MOTOR_NUM + i];
                            double target_vel = single_gomotor_command_shared[VELOCITY_TARGET_IDX*MOTOR_NUM + i];
                            double kp = single_gomotor_command_shared[P_GAIN_IDX*MOTOR_NUM + i];
                            double kd = single_gomotor_command_shared[D_GAIN_IDX*MOTOR_NUM + i];
                            kp = 16.0;
                            kd = 1.2;

                            #if USE_ACCELELERATION_TARGET_FLAG
                            double shm_acc_set_time_ctrl_clock = single_gomotor_command_shared[ACCELERATION_SET_CLOCK_TIME_IDX*MOTOR_NUM + i];
                            shm_acc_set_time_ctrl_clock = cycle_start_us/1000000.0;
                            //
                            if(std::abs(shm_acc_set_time_ctrl_clock)>1e-8)
                            {
                                // [pos_accref mode]
                                // over-write target_pos and target_vel based on accref
                                struct timespec ts_now;
                                clock_gettime(CLOCK_MONOTONIC, &ts_now);
                                double tmp_soem_clock = ts_now.tv_sec + 0.000000001*ts_now.tv_nsec;
                                if (cycle_num == 0) {
                                    control_start_clock = tmp_soem_clock;
                                }
                                double target_acc = single_gomotor_command_shared[ACCELERATION_TARGET_IDX*MOTOR_NUM + i];
                                target_acc = acc_amplitude * sin(2.0*M_PI*acc_frequency*(tmp_soem_clock - control_start_clock));
                                // check if acc_set_time is updated
                                if(std::abs(last_acc_set_time_ctrl_clock[i] - shm_acc_set_time_ctrl_clock)>1e-5)
                                {
                                    //std::cout<<"\n[debug print] acc_set_time_soem_clock[i]: "<<acc_set_time_soem_clock[i]<<"\n"<<std::endl;
                                    last_acc_set_time_ctrl_clock[i] = shm_acc_set_time_ctrl_clock;
                                    acc_set_time_soem_clock[i] = tmp_soem_clock;
                                    // [type 1]: give initial values by sensor data
                                    // pos_at_acc_set_time[i] = single_gomotor_sensor_shared[POSITION_OBS_IDX*MOTOR_NUM + i];
                                    // vel_at_acc_set_time[i] = single_gomotor_sensor_shared[VELOCITY_OBS_IDX*MOTOR_NUM + i];
                                    // [type 2]: give initial values by last ref
                                    pos_at_acc_set_time[i] = recent_pos_ref_only_used_by_accref[i];
                                    vel_at_acc_set_time[i] = recent_vel_ref_only_used_by_accref[i];
                                }

                                double dt_ = tmp_soem_clock - acc_set_time_soem_clock[i] + 0.0005; // add 0.5 ms
                                target_vel = vel_at_acc_set_time[i] + target_acc*dt_;
                                target_pos = pos_at_acc_set_time[i] + vel_at_acc_set_time[i]*dt_ + 0.5*target_acc*dt_*dt_;
                                // from Motor Catalogue
                                double max_vel = 30.0; // rad/s
                                target_vel = std::max(-max_vel, std::min(max_vel, target_vel));
                                log_cmd_data(tmp_soem_clock, shm_acc_set_time_ctrl_clock, target_acc, recent_pos_ref_only_used_by_accref[i], recent_vel_ref_only_used_by_accref[i], target_pos, target_vel);
                                log_obs_data(single_gomotor_sensor_shared[OBS_GET_CLOCK_TIME_IDX],
                                                single_gomotor_sensor_shared[POSITION_OBS_IDX],
                                                single_gomotor_sensor_shared[VELOCITY_OBS_IDX] );
                                recent_pos_ref_only_used_by_accref[i] = 1.0 * target_pos;
                                recent_vel_ref_only_used_by_accref[i] = 1.0 * target_vel;
                            }
                            else
                            {
                                struct timespec ts_now;
                                clock_gettime(CLOCK_MONOTONIC, &ts_now);
                                double tmp_soem_clock = ts_now.tv_sec + 0.000000001*ts_now.tv_nsec;
                                double target_acc = 0.0;
                                // [trq mode, pos mode, zero-cmd mode]

                                // Here, we do not overwrite target_pos and target_vel.

                                // What we do here is resetting variables only used by accref mode.
                                last_acc_set_time_ctrl_clock[i] = 0.0;
                                log_cmd_data(tmp_soem_clock, shm_acc_set_time_ctrl_clock, target_acc, recent_pos_ref_only_used_by_accref[i], recent_vel_ref_only_used_by_accref[i], target_pos, target_vel);
                                log_obs_data(single_gomotor_sensor_shared[OBS_GET_CLOCK_TIME_IDX],
                                                single_gomotor_sensor_shared[POSITION_OBS_IDX],
                                                single_gomotor_sensor_shared[VELOCITY_OBS_IDX] );
                                recent_pos_ref_only_used_by_accref[i] = single_gomotor_sensor_shared[POSITION_OBS_IDX*MOTOR_NUM + i];
                                recent_vel_ref_only_used_by_accref[i] = single_gomotor_sensor_shared[VELOCITY_OBS_IDX*MOTOR_NUM + i];
                            }
                            #endif

                            double torque_max = 23.5;
                            torque_control = std::max(-torque_max, std::min(torque_max, torque_control));
                            /*指令値セット*/
                            #if (ENABLE_SINGLE_GOMOTOR == 1)
                            set_mode(1, motor[i].send);
                            set_torque(torque_control / 6.33, motor[i].send);
                            set_position((target_pos) * 6.33, motor[i].send);
                            set_speed(target_vel * 6.33, motor[i].send);
                            set_K_P(kp / ( 6.33 * 6.33 ), motor[i].send);
                            set_K_W(kd / ( 6.33 * 6.33 ), motor[i].send);
                            /***********************/
                            // st_clock[i] = clock();
                            clock_gettime(CLOCK_MONOTONIC, &t_st[i]);
                            set_output(1, i, motor[i].send);
                            #endif
                        }
                        if (cycle_num < buf_size) {
                            check_buf[0][cycle_num][i] = check[i];
                        }
		            }
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);
                    if (wkc >= expectedWKC) {
                        for (int cnt = 0; cnt < MOTOR_NUM; cnt++) {
                            if (cycle_num < buf_size) {
                                check_buf[1][cycle_num][cnt] = *(motor[cnt].recv + 14);
                            }
			                if (check[cnt] == *(motor[cnt].recv + 14)) {
                                // uint32_t g_tim6_irq_count = get_g_tim6_irq_count(motor[cnt].recv);
                                // printf("\033[%d;1H", monitoring_print_cursor + 2);
                                // printf("\033[0K");
                                // printf("check %u: g_tim6_irq_count %u\n", check[cnt], g_tim6_irq_count);
                                // printf("\033[0K");
			                // if (true) {
                                // end_clock[cnt] = clock();
                                clock_gettime(CLOCK_MONOTONIC, &t_end[cnt]);
                                recv_fin[cnt] = TRUE;
                                if (check[cnt] == 0) {
                                    check[cnt] = 255;
                                } else {
                                    check[cnt]--;
                                }
                                /* 受信データ表示
                                    id      :モータナンバー(unitreeのidとは違うもの)
                                    torque  :トルク
                                    anglevel:角速度
                                    angle   :角度
                                    temp    :温度
                                */
                                if (check_CRC(motor[cnt].recv)) {  // CRCチェック
                                    double raw_position = get_position(motor[cnt].recv);
                                    single_gomotor_sensor_shared[POSITION_OBS_IDX*MOTOR_NUM + cnt] = raw_position / 6.33;
                                    single_gomotor_sensor_shared[VELOCITY_OBS_IDX*MOTOR_NUM + cnt] = get_angular_vel(motor[cnt].recv) / 6.33;
                                    single_gomotor_sensor_shared[TORQUE_OBS_IDX*MOTOR_NUM + cnt] = get_torque(motor[cnt].recv) * 6.33;
                                    single_gomotor_sensor_shared[TEMPERATURE_OBS_IDX*MOTOR_NUM + cnt] = get_temp(motor[cnt].recv);
                                    single_gomotor_sensor_shared[OBS_GET_CLOCK_TIME_IDX*MOTOR_NUM + cnt] = t_end[cnt].tv_sec + 0.000000001*t_end[cnt].tv_nsec;
				                    {
                                        printf("\033[%d;1H", cnt + monitoring_print_cursor);
                                        char message[20];
                                        printf("\033[0K");
                                        printf("id: %2d, angle: %12.6lf(rad), anglevel: %12.6lf(rad/s), torque: %10.6lf(Nm), temp: %3f℃ , error: %s\n", cnt,
                                        single_gomotor_sensor_shared[POSITION_OBS_IDX*MOTOR_NUM + cnt], single_gomotor_sensor_shared[VELOCITY_OBS_IDX*MOTOR_NUM + cnt], single_gomotor_sensor_shared[TORQUE_OBS_IDX*MOTOR_NUM + cnt], single_gomotor_sensor_shared[TEMPERATURE_OBS_IDX*MOTOR_NUM + cnt],
                                        check_err(motor[cnt].recv, message));
                                        printf("\033[%d;1H", cnt + monitoring_print_cursor + MOTOR_NUM);
                                        printf("\033[0K");
				                    }
                                    time_count[cnt][time_index[cnt]] = (double)(t_end[cnt].tv_nsec - t_st[cnt].tv_nsec) / 1000000;
                                    if (time_count[cnt][time_index[cnt]] < 0) {
                                        time_count[cnt][time_index[cnt]] += 1000;
                                    }
                                    now_time[cnt] = time_index[cnt];
                                    time_index[cnt]++;
                                    if (time_index[cnt] == NUM) {
                                        time_index[cnt] = 0;
                                        mesure(time_count[cnt], &(ave_time[cnt]), &(var_time[cnt]), &(max_time[cnt]), &(over_num[cnt]), &over_num2[cnt], &min_time[cnt]);
                                    }
                                }
                                else
                                {
                                    // printf("\033[%d;1H", MOTOR_NUM + 12 + cnt);
                                    printf("cycle %d, id %d CRC_error", cycle_num, cnt);
                                    // printf("\a");
                                    // check[cnt]++;
                                }
			                }
                        } // End of for (int cnt = 0; cnt < MOTOR_NUM; cnt++)
                        proc_comm_sensor->write_stdvec(single_gomotor_sensor_shared);
                        //計測時間表示
                        /*
                        {
                          for (int cnt = 0; cnt < MOTOR_NUM; cnt++) {
                            printf("\033[%d;1H", MOTOR_NUM + monitoring_print_cursor + cnt + 5);
                            printf("\033[0K");
                            printf("id %d: time %fms ,", cnt, time_count[cnt][now_time[cnt]]);
                            printf("ave %8.6fms ,var %8.6fms ,max %8.6fms ,min %8.6fms ,over ratio(%5.2lfkHz) %7.4f %% ,over ratio(%5.2lfkHz) %7.4f %%\n", ave_time[cnt], var_time[cnt], max_time[cnt], min_time[cnt], 1.0 / (float)TH, (float)over_num[cnt] / (float)NUM * 100.0, 1.0 / (float)TH2, (float)over_num2[cnt] / (float)NUM * 100.0);
                          }
                        }
                        */
                        needlf = TRUE;
                    } // End of if (wkc >= expectedWKC)
                    else
                    {
                        printf("\033[%d;1H", monitoring_print_cursor + 4);
                        printf("\033[0K");
                        printf("wkc error %d\n", wkc);
                    }
                    if (0==keepRunning) break;
                    // osal_usleep(50);
                    cycle_end_us = get_time_us();
                    if (cycle_num < buf_size) {
                        for (int cnt = 0; cnt < MOTOR_NUM; cnt++) {
                            time_buf[0][cycle_num][cnt] = cycle_start_us;
                            time_buf[1][cycle_num][cnt] = cycle_end_us;
                            time_buf[2][cycle_num][cnt] = cycle_end_us - cycle_start_us;
                        }
                        cycle_num++;
                    } else if (cycle_num == buf_size) {
                        
                        for (int cnt = 0; cnt < MOTOR_NUM; cnt++) {
                            std::ofstream logging_file("ethercat_master_logging.csv");
                            logging_file << "cycle" << ","
                                        << "check_order" << ","
                                        << "check(slave)" << ","
                                        << "cycle_start_us" << ","
                                        << "cycle_end_us" << ","
                                        << "cycle_elapsed_us" << "\n";
                            for (int i = 0; i < buf_size; i++) {
                                logging_file << i << ","
                                            << static_cast<int>(check_buf[0][i][cnt]) << ","
                                            << static_cast<int>(check_buf[1][i][cnt]) << ","
                                            << time_buf[0][i][cnt] << ","
                                            << time_buf[1][i][cnt] << ","
                                            << time_buf[2][i][cnt] << "\n";
                            }
                            logging_file.close();
                        }
                        save_log_to_file();
                        cycle_num++;
                    }
                } // End of cyclic loop
                inOP = FALSE;
                // save_log_to_file();
            } // End of if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++) {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("\n\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        } // End of if (ec_config_init(FALSE) > 0)
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        printf("keepRunning = %d \n",keepRunning);

        // ゼロ指令値を送信して終了
        for (int i = 0; i < MOTOR_NUM; i++) {
          if (recv_fin[i]) {
            recv_fin[i] = FALSE;
            motor[i].send[15] = check[i];
            /*指令値セット*/
            set_mode(1, motor[i].send);
            set_torque(0, motor[i].send);
            set_speed(0, motor[i].send);
            set_K_P(0, motor[i].send);
            set_K_W(0, motor[i].send);
            set_position(0, motor[i].send);
            set_output(1, i, motor[i].send);
          }
        }

        /* stop SOEM, close socket */
        ec_close();
    } // End of if (ec_init(ifname))
    else
    {
        printf("No socket connection on %s\nExecute as root\n", ifname);
    }

    delete proc_comm_sensor;
    delete proc_comm_command;
}

OSAL_THREAD_FUNC ecatcheck(void* ptr)
{
    int slave;
    (void)ptr; /* Not used */

    while (1) {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
            if (needlf) {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++) {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        //printf("\033[%d;1H", MOTOR_NUM + 20);
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
                        //printf("\033[%d;1H", MOTOR_NUM + 20);
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    } else if (ec_slave[slave].state > EC_STATE_NONE) {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
                            ec_slave[slave].islost = FALSE;
                            //printf("\033[%d;1H", MOTOR_NUM + 20);
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    } else if (!ec_slave[slave].islost) {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE) {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost) {
                    if (ec_slave[slave].state == EC_STATE_NONE) {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    } else {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            // if (!ec_group[currentgroup].docheckstate)
            //     printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

void dry_run_single_gomotor()
{
  std::cout<<"start void dry_run_single_gomotor()"<<std::endl;

  int is_host = 1;
  ProcComm *proc_comm_sensor;
  ProcComm *proc_comm_command;
  proc_comm_sensor = new ProcComm(filename_data_single_gomotor_sensor, id_data_single_gomotor_sensor, num_data_single_gomotor_sensor, is_host);
  proc_comm_command = new ProcComm(filename_data_single_gomotor_command, id_data_single_gomotor_command, num_data_single_gomotor_command, is_host);

  while(keepRunning)
  {
    single_gomotor_command_shared = proc_comm_command->read_stdvec();
    std::cout<<"[dry run] single_gomotor_command: ";
    for(int i=0;i<single_gomotor_command_shared.size();i++)
    {
      std::cout<<single_gomotor_command_shared[i]<<", ";
    }
    std::cout<<std::endl;
    sleep(1);
  }
  printf("keepRunning = %d \n",keepRunning);

  delete proc_comm_sensor;
  delete proc_comm_command;
}

int main(int argc, char* argv[])
{
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    std::cout<<"ENABLE_SINGLE_GOMOTOR: "<<ENABLE_SINGLE_GOMOTOR<<std::endl;

    signal(SIGINT, signal_handler); // killed by ctrl+C
    signal(SIGHUP, signal_handler); // killed by tmux kill-server

    #if (ENABLE_SINGLE_GOMOTOR > 0)
    if (argc > 1) {
        /* create thread to handle slave error handling in OP */
        // osal_thread_create(&thread1, 128000, &ecatcheck, NULL);
        osal_thread_create(&thread1, 128000, (void *)&ecatcheck, NULL);
        /* start cyclic part */
        simpletest(argv[1]);
    } else {
        ec_adaptert* adapter = NULL;
        printf("Usage: simple_test ifname1\nifname = eth0 for example\n");

        printf("\nAvailable adapters:\n");
        adapter = ec_find_adapters();
        while (adapter != NULL) {
            printf("    - %s  (%s)\n", adapter->name, adapter->desc);
            adapter = adapter->next;
        }
        ec_free_adapters(adapter);
    }
    #endif

    #if (ENABLE_SINGLE_GOMOTOR == 0)
    dry_run_single_gomotor();
    #endif

    printf("End program\n");
    return (0);
}
