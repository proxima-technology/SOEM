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
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <linux/types.h>
#include <linux/module.h>
#include "set.h"
#include "get.h"

#define EC_TIMEOUTMON 500
#define NUM 1000
#define TH 0.5
#define TH2 0.333333
#define MOTOR_NUM 1

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
    for (int i = 0; i < MOTOR_NUM; i++) {
        set_id(0, motor[i].send);
    }
    // set_id(0, motor[0].send);
    // set_id(1, motor[1].send);
    // // set_id(0, motor[2].send);
    // // set_id(0, motor[3].send);
    // set_id(1, motor[4].send);
    // set_id(1, motor[5].send);

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

#define obs_log_data_per_step 3
#define cmd_log_data_per_step 7
// ロギング用のバッファサイズ
#define log_buffer_size 30000
// ロギング用のバッファ
double obs_log_buffer[log_buffer_size][obs_log_data_per_step];
double cmd_log_buffer[log_buffer_size][cmd_log_data_per_step];
// ロギング用のバッファのインデックス
int obs_log_index = 0;
int cmd_log_index = 0;
// ロギング用のファイル名
FILE* obs_log_file;
const char* obs_log_filepath = "single_gomotor_obs_log.csv";
// std::string obs_log_filename = "single_gomotor_obs_log.csv";
// std::string cmd_log_filename = "single_gomotor_cmd_log.csv";
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
    obs_log_file = fopen(obs_log_filepath, "w");

    // 観測データのログをファイルに保存


    for (int i = 0; i < obs_log_index; i++) {
        fprintf(obs_log_file, "%.5f,%.5f,%.5f\n", obs_log_buffer[i][0], obs_log_buffer[i][1], obs_log_buffer[i][2]);
    }
    // 指令データのログをファイルに保存
    // for (int i = 0; i < cmd_log_index; i++) {
    //     cmd_log_file << cmd_log_buffer[i][0] << ","
    //                  << cmd_log_buffer[i][1] << ","
    //                  << cmd_log_buffer[i][2] << ","
    //                  << cmd_log_buffer[i][3] << ","
    //                  << cmd_log_buffer[i][4] << ","
    //                  << cmd_log_buffer[i][5] << ","
    //                  << cmd_log_buffer[i][6] << "\n";
    // }
    // ファイルを閉じる
    fclose(obs_log_file);
    // cmd_log_file.close();
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

/*
    ethercat通信するスレッド
*/
void simpletest(char* ifname)
{
    int i, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

    uint8 check[MOTOR_NUM];
    uint8 check_order[MOTOR_NUM];
    volatile bool recv_fin[MOTOR_NUM];
    for (int i = 0; i < MOTOR_NUM; i++) {
        check[i] = 255;
        check_order[i] = 0;
        recv_fin[i] = TRUE;
    }
    double time_count[MOTOR_NUM][NUM];
    int time_index[MOTOR_NUM] = {0};
    // int now_time[MOTOR_NUM] = {0};
    double max_time[MOTOR_NUM] = {0};
    double min_time[MOTOR_NUM] = {0};
    double ave_time[MOTOR_NUM] = {0};
    double var_time[MOTOR_NUM] = {0};
    int over_num[MOTOR_NUM] = {0};
    int over_num2[MOTOR_NUM] = {0};

    uint8* data_prev[MOTOR_NUM];
    // int nodata[MOTOR_NUM] = {0};
    for (int i = 0; i < MOTOR_NUM; i++) {
        data_prev[i] = calloc(14, sizeof(uint8));
    }

    printf("\033[2J\033[1;1H");  // 画面クリア
    printf("Starting simple test\n");

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
            if (oloop > 128)
                oloop = 128;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1;
            if (iloop > 128)
                iloop = 128;

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
            do {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
                printf("Operational state reached for all slaves.\n");

                // check the version id of the slave

                const char vcheck[] = "vcheck";
                ec_slave[1].outputs[15] = 0xcc;
                int hash_check_cnt = 0;
                do {
                    ec_send_processdata();
                    ec_receive_processdata(EC_TIMEOUTRET);
                    hash_check_cnt++;
                } while (strncmp(vcheck, ec_slave[1].inputs, sizeof(vcheck) / sizeof(vcheck[0])) != 0 && hash_check_cnt < 1000);

                int verlen = ec_slave[1].inputs[sizeof(vcheck) / sizeof(vcheck[0])];
                uint8_t* slave_ver;
                slave_ver = (uint8_t*)malloc(verlen * sizeof(uint8_t));
                memcpy(slave_ver, ec_slave[1].inputs + sizeof(vcheck) / sizeof(vcheck[0]) + 1, verlen);
                printf("slave commit ID is %s\n", slave_ver);
                ec_slave[1].outputs[15] = 0x00;
                free(slave_ver);

                printf("\033[10;1H");
                printf("motor INFO");
                printf("\033[%d;1H", MOTOR_NUM + 13);
                printf("data transmission speed");

                inOP = TRUE;
                set_init();

                for (uint i = 0; i < MOTOR_NUM; i++) {
                    motor[i].recv = get_recv_pointer(i);
                }
                // clock_t cyc_f = 0, cyc_f_pre = 0;
                // float tor = 0;
                // float tor2 = 0;
                struct timespec t_st[MOTOR_NUM], t_end[MOTOR_NUM];
                volatile bool first_come[MOTOR_NUM] = {false};
                uint8_t order_prev[15];
                volatile int badcnt = 0;
                volatile int goodcnt = 0;
                float tor = 0;
                float d_tor = 0.01;

                /* cyclic loop */
                for (int loop_cnt = 0; loop_cnt < NUM * 10; loop_cnt++) {
                    for (int i = 0; i < MOTOR_NUM; i++) {
                        if (recv_fin[i]) {
                            recv_fin[i] = FALSE;
                            memcpy(order_prev, motor[i].send, 15);
                            check_order[i]++;
                            if (check_order[i] >= 0xcc) {
                                check_order[i] = 0;
                            }
                            motor[i].send[15] = check_order[i];
                            /*指令値セット*/
                            set_mode(1, motor[i].send);

                            if (tor > 0.05 || tor < -0.05) {
                                d_tor *= -1;
                            }
                            tor += d_tor;
                            set_torque(tor, motor[i].send);
                            set_speed(0, motor[i].send);
                            set_K_P(0, motor[i].send);
                            set_K_W(0, motor[i].send);
                            set_position(0, motor[i].send);
                            /***********************/

                            clock_gettime(CLOCK_MONOTONIC, &t_st[i]);

                            set_output(1, i, motor[i].send);
                        }
                    }

                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);
                    if (wkc >= expectedWKC) {
                        for (int cnt = 0; cnt < MOTOR_NUM; cnt++) {
                            // if (*(motor[cnt].recv + 14)) {
                            if (*(motor[cnt].recv + 14) != check[cnt]) {
                                printf("\033[%d;1H\033[0K", 20 + cnt);
                                // printf("%d", check[cnt]);
                                /*
                                if (*(motor[cnt].recv + 14) < check[cnt]) {
                                    printf("cnt_num%d", *(motor[cnt].recv + 14) - check[cnt] + 255);
                                } else {
                                    printf("cnt_num%d", *(motor[cnt].recv + 14) - check[cnt]);
                                }
                                */

                                //if (memcmp(data_prev[cnt], motor[cnt].recv, 14 * sizeof(uint8)) == 0) {
                                // printf("\033[%d;1H\033[0K", 31 + cnt);
                                // printf("id %d no change %d\n", cnt, nodata[cnt]++);
                                // continue;
                                //}

                                if (memcmp(order_prev, motor[cnt].recv + 15, 15 * sizeof(uint8)) != 0) {
                                    badcnt++;
                                    printf("\033[%d;1H\033[0K", 40 + cnt);
                                    // for(int j = 0; j <15;j++){
                                    // printf("%2x %2x %2lf\n", *(motor[cnt].recv + 16), order_prev[1], d_tor);
                                    //}
                                    if (d_tor > 0) {
                                        if (order_prev[1] > *(motor[cnt].recv + 16)) {
                                            printf("PC  fast+\n");
                                        } else {
                                            printf("stm fast+\n");
                                        }
                                    } else if (d_tor < 0) {
                                        if (order_prev[1] < *(motor[cnt].recv + 16)) {
                                            printf("PC  fast-\n");
                                        } else {
                                            printf("stm fast-\n");
                                        }
                                    }
                                } else {
                                    goodcnt++;
                                }
                                // printf("\033[%d;1H\033[0K", 38 + cnt);
                                // printf("incorrect %d correct %d\n", badcnt, goodcnt);

                                memcpy(data_prev[cnt], motor[cnt].recv, 14 * sizeof(uint8));
                                check[cnt] = *(motor[cnt].recv + 14);
                                clock_gettime(CLOCK_MONOTONIC, &t_end[cnt]);
                                recv_fin[cnt] = TRUE;
                                /*
                                    受信データ表示
                                    id      :モータナンバー(unitreeのidとは違うもの)
                                    torque  :トルク
                                    anglevel:角速度
                                    angle   :角度
                                    temp    :温度
                                */
                                if (check_CRC(motor[cnt].recv)) {  // CRCチェック
                                    char message[20];
                                    /*フィードバック値表示*/
                                    //printf("\033[%d;1H\033[0K", cnt + 12);
                                    //printf("id: %2d, torque: %10.6lf(Nm), anglevel: %12.6lf(rad/s), angle: %12.6lf(rad), temp: %3d℃ , error: %s\n", cnt, get_torque(motor[cnt].recv), get_angular_vel(motor[cnt].recv), get_position(motor[cnt].recv), get_temp(motor[cnt].recv), check_err(motor[cnt].recv, message));
                                    //printf("\033[%d;1H\033[0K", MOTOR_NUM + 15 + cnt);
                                    //printf("id: %2d, ave %8.6fms ,var %8.6fms ,max %8.6fms ,min %8.6fms ,over ratio(%4.1lfkHz) %4.1f %% ,over ratio(%4.1lfkHz) %4.1f %%\n", cnt, ave_time[cnt], var_time[cnt], max_time[cnt], min_time[cnt], 1.0 / (float)TH, (float)over_num[cnt] / (float)NUM * 100.0, 1.0 / (float)TH2, (float)over_num2[cnt] / (float)NUM * 100.0);
                                    time_count[cnt][time_index[cnt]] = (double)(t_end[cnt].tv_nsec - t_st[cnt].tv_nsec) / 1000000;
                                    if (time_count[cnt][time_index[cnt]] < 0) {
                                        time_count[cnt][time_index[cnt]] += 1000;
                                    }
                                    // now_time[cnt] = time_index[cnt];
                                    time_index[cnt]++;
                                    if (time_index[cnt] == NUM) {
                                        time_index[cnt] = 0;
                                        mesure(time_count[cnt], &(ave_time[cnt]), &(var_time[cnt]), &(max_time[cnt]), &(over_num[cnt]), &over_num2[cnt], &min_time[cnt]);
                                        /*フィードバック値表示*/
                                        printf("\033[%d;1H\033[0K", cnt + 12);
                                        printf("id: %2d, torque: %10.6lf(Nm), anglevel: %12.6lf(rad/s), angle: %12.6lf(rad), temp: %3d℃ , error: %s\n", cnt, get_torque(motor[cnt].recv), get_angular_vel(motor[cnt].recv), get_position(motor[cnt].recv), get_temp(motor[cnt].recv), check_err(motor[cnt].recv, message));
                                        /*計測時間表示*/
                                        printf("\033[%d;1H\033[0K", MOTOR_NUM + 15 + cnt);
                                        printf("id: %2d, ave %8.6fms ,var %8.6fms ,max %8.6fms ,min %8.6fms ,over ratio(%4.1lfkHz) %4.1f %% ,over ratio(%4.1lfkHz) %4.1f %%\n", cnt, ave_time[cnt], var_time[cnt], max_time[cnt], min_time[cnt], 1.0 / (float)TH, (float)over_num[cnt] / (float)NUM * 100.0, 1.0 / (float)TH2, (float)over_num2[cnt] / (float)NUM * 100.0);
                                        printf("\033[%d;1H\033[0K", 38 + cnt);
                                        printf("incorrect %d correct %d\n", badcnt, goodcnt);
                                        printf("rs485 %d0us", motor[cnt].recv[31]);
                                    }
                                } else {
                                    if (first_come[cnt]) {
                                        printf("\033[%d;1H\033[0K", 12 + cnt);
                                        printf("id %d CRC_error", cnt);
                                    } else {
                                        first_come[cnt] = true;
                                    }
                                }
                            }
                            log_obs_data(get_torque(motor[cnt].recv), get_position(motor[cnt].recv), get_angular_vel(motor[cnt].recv));
                        }
                        needlf = TRUE;

                    } else {
                        static int i = 0;
                        printf("\033[40;1H%2d ", wkc);
                        printf("wkc error %3d\n", i);
                        i++;
                        if (i > 256) {
                            i = 0;
                        }
                    }
                    osal_usleep(300);
                }
                inOP = FALSE;
                save_log_to_file();
            } else {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++) {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        } else {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
        for (int i = 0; i < MOTOR_NUM; i++) {
            free(data_prev[i]);
        }
    } else {
        printf("No socket connection on %s\nExecute as root\n", ifname);
    }
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

int main(int argc, char* argv[])
{
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    if (argc > 1) {
        /* create thread to handle slave error handling in OP */
        osal_thread_create(&thread1, 128000, &ecatcheck, NULL);
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

    printf("End program\n");
    return (0);
}
