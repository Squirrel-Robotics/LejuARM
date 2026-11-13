#!/bin/sh
### BEGIN INIT INFO
# Provides:          can_add_server.sh
# Required-start:    $local_fs $remote_fs $network $syslog
# Required-Stop:     $local_fs $remote_fs $network $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: init can
# Description:       init can
### END INIT INFO

# 等待 udev 把 SPI CAN 接口(bcan0~bcan3)重命名完成再配置
# 解决新 boot.img 下开机竞态：接口约 5.8s 才出现，原 sleep 0.5 太短导致 ip link 报 "Cannot find device"
for i in $(seq 1 60); do
    if ip link show bcan0 >/dev/null 2>&1 && ip link show bcan1 >/dev/null 2>&1 && ip link show bcan2 >/dev/null 2>&1 && ip link show bcan3 >/dev/null 2>&1; then
        echo "CAN 接口已就绪 (等待约 $((i/2))s)"
        break
    fi
    [ "$i" -eq 60 ] && echo "警告: 等待 bcan0~bcan3 出现超时(30s)，继续尝试配置"
    sleep 0.5
done

CAN_NAME_PREFIX="bcan"

sudo ip link set ${CAN_NAME_PREFIX}0 type can bitrate 1000000 sample-point 0.80 dbitrate 5000000 dsample-point 0.750 fd on loopback off
sudo ip link set up ${CAN_NAME_PREFIX}0
sudo ip link set ${CAN_NAME_PREFIX}1 type can bitrate 1000000 sample-point 0.80 dbitrate 5000000 dsample-point 0.750 fd on loopback off
sudo ip link set up ${CAN_NAME_PREFIX}1
#sudo ip link set ${CAN_NAME_PREFIX}0 type can bitrate 1000000 sample-point 0.800 loopback off
#sudo ip link set up ${CAN_NAME_PREFIX}0
#sudo ip link set ${CAN_NAME_PREFIX}1 type can bitrate 1000000 sample-point 0.800 loopback off
#sudo ip link set up ${CAN_NAME_PREFIX}1
sudo ip link set ${CAN_NAME_PREFIX}2 type can bitrate 1000000 sample-point 0.800 loopback off
sudo ip link set up ${CAN_NAME_PREFIX}2
sudo ip link set ${CAN_NAME_PREFIX}3 type can bitrate 1000000 sample-point 0.800 loopback off
sudo ip link set up ${CAN_NAME_PREFIX}3
#sudo ip link set ${CAN_NAME_PREFIX}2 type can bitrate 1000000 sample-point 0.80 dbitrate 5000000 dsample-point 0.750 fd on loopback off
#sudo ip link set up ${CAN_NAME_PREFIX}2
#sudo ip link set ${CAN_NAME_PREFIX}3 type can bitrate 1000000 sample-point 0.80 dbitrate 5000000 dsample-point 0.750 fd on loopback off
#sudo ip link set up ${CAN_NAME_PREFIX}3

sudo ifconfig ${CAN_NAME_PREFIX}0 txqueuelen 5000
sudo ifconfig ${CAN_NAME_PREFIX}1 txqueuelen 5000
sudo ifconfig ${CAN_NAME_PREFIX}2 txqueuelen 5000
sudo ifconfig ${CAN_NAME_PREFIX}3 txqueuelen 5000


echo "=== 绑定 CAN 中断线程 ==="

can0_pid=$(ps aux | grep "irq/93-bcan0" | grep -v grep | awk '{print $2}')
if [ ! -z "$can0_pid" ]; then
    echo "绑定 bcan0 (PID $can0_pid) 到 CPU 4"
    taskset -cp 4 $can0_pid
fi

can1_pid=$(ps aux | grep "irq/94-bcan1" | grep -v grep | awk '{print $2}')
if [ ! -z "$can1_pid" ]; then
    echo "绑定 bcan1 (PID $can1_pid) 到 CPU 5"
    taskset -cp 4 $can1_pid
fi

can2_pid=$(ps aux | grep "irq/95-bcan2" | grep -v grep | awk '{print $2}')
if [ ! -z "$can2_pid" ]; then
    echo "绑定 bcan2 (PID $can2_pid) 到 CPU 6"
    taskset -cp 5 $can2_pid
fi

can3_pid=$(ps aux | grep "irq/96-bcan3" | grep -v grep | awk '{print $2}')
if [ ! -z "$can3_pid" ]; then
    echo "绑定 bcan3 (PID $can3_pid) 到 CPU 7"
    taskset -cp 5 $can3_pid
fi

spi1_pid=$(ps aux | grep "spi1" | grep -v grep | awk '{print $2}')
if [ ! -z "$spi1_pid" ]; then
    echo "绑定 spi1 (PID $spi1_pid) 到 CPU 5"
    taskset -cp 4 $spi1_pid
fi

spi4_pid=$(ps aux | grep "spi4" | grep -v grep | awk '{print $2}')
if [ ! -z "$spi4_pid" ]; then
    echo "绑定 spi4 (PID $spi4_pid) 到 CPU 7"
    taskset -cp 5 $spi4_pid
fi

kworker_pid=$(ps aux | grep "kworker/.*mcan_wq" | grep -v grep | awk '{print $2}')
if [ ! -z "$kworker_pid" ]; then
    for pid in $kworker_pid; do
        echo "绑定工作队列 (PID $pid) 到 CPU 6"
        taskset -cp 5 $pid
    done
fi

sleep 1
echo "=== 绑定完成 ==="
ps -eo pid,comm,psr | grep -E "irq/.*can|spi|mcan_wq"


exit 0

