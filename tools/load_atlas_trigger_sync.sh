#!/usr/bin/env bash
set -euo pipefail

# Atlas hardware mapping: GPIO4_B6_d (PCIe30X4_PERSTn_M1_L) is Linux GPIO 142.
# GPIO mode: the kernel module drives both the camera trigger and radar PPS.
MODULE=/userdata/atlas/kernel/imx586_trigger_sync/prebuilt/6.1.141/imx586_trigger_sync.ko
PPS_GPIO_PIN=${PPS_GPIO_PIN:-142}
# Current calibrated phase: exposure-center offset (12.888975 ms at 919 lines)
# minus the measured radar-minus-camera residual (about 7.2 ms).
PPS_PHASE_NS=${PPS_PHASE_NS:-5689000}
if [[ -e /sys/module/imx586_trigger_sync ]]; then
  current=$(cat /sys/module/imx586_trigger_sync/parameters/pps_gpio_pin)
  pps_enabled=$(cat /sys/module/imx586_trigger_sync/parameters/enable_pps_gpio)
  [[ "$current" == "$PPS_GPIO_PIN" ]] || {
    echo "imx586_trigger_sync is already loaded with pps_gpio_pin=$current" >&2
    exit 1
  }
  [[ "$pps_enabled" == "Y" || "$pps_enabled" == "1" ]] || {
    echo "imx586_trigger_sync is not driving PPS GPIO; reload with enable_pps_gpio=1" >&2
    exit 1
  }
  echo "imx586_trigger_sync already loaded in GPIO PPS mode"
  exit 0
else
  insmod "$MODULE" \
    camera_gpio_pin=54 \
    pps_gpio_pin="$PPS_GPIO_PIN" \
    enable_pps_gpio=1 \
    camera_period_ns=100000000 \
    camera_pulse_width_ns=5000000 \
    pps_period_ns=1000000000 \
    pps_pulse_width_ns=10000000 \
    pps_phase_ns="$PPS_PHASE_NS"
fi
echo "loaded imx586_trigger_sync: camera_gpio_pin=54 pps_gpio_pin=$PPS_GPIO_PIN enable_pps_gpio=1"
