#!/bin/bash

# SOEM単体テスト
sudo setcap cap_net_raw=eip ./build/test/linux/cat_test_4/cat_test_4
./build/test/linux/cat_test_4/cat_test_4 enx6c1ff7134a40

# 共有メモリを組み込んだ通信プログラムのテスト
# sudo setcap cap_net_raw=eip ./build/test/linux/proxima_robot/proxima_robot
# ./build/test/linux/proxima_robot/proxima_robot enp46s0
