#!/bin/bash

# 最初のオンラインCPUから利用可能な最大周波数を抽出
MAX_FREQUENCY=$(awk '{print $1}' /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_frequencies)

# CPUのスケーリングガバナーを userspace に設定
for i in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  echo "userspace" | sudo tee $i;
done

# 各CPUコアの周波数を最大周波数に設定
for i in /sys/devices/system/cpu/cpu*/cpufreq/scaling_setspeed; do
  echo "$MAX_FREQUENCY" | sudo tee $i;
done

sudo sh -c "echo 0 > /sys/devices/system/cpu/cpufreq/boost"
