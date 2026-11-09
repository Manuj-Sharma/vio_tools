#!/usr/bin/env bash
# Stop everything started by vio_live_with_filter.sh (OpenVINS, RViz, filter node, sensors). Lives in a file so the pkill -f
# patterns cannot match the caller's command line.
pkill -INT -f "[r]un_openvins_mono.sh"
sleep 2
pkill -x run_subscribe_m
pkill -x rviz2
pkill -x imu_filter_node
pkill -INT -f "[v]io_bringup.launch.py"
sleep 2
pkill -x vizionsdk_camer
pkill -x imx5_spi_node
pkill -f "[f]light_dataset_recorder_publisher.py"
sleep 2
for p in run_subscribe_m rviz2 imu_filter_node vizionsdk_camer imx5_spi_node; do
  pgrep -x "$p" >/dev/null && echo "STILL running: $p" || echo "stopped: $p"
done
