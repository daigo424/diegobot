#!/usr/bin/env bash
# ラズパイ本体の電源・温度・クロック状態をvcgencmdでまとめて表示する。
# make pi-diagnose から呼ばれる想定(SSH経由でこのスクリプトをそのまま実行する)。

set -euo pipefail

echo "=== 電圧・スロットリング状態 (vcgencmd get_throttled) ==="
RAW=$(vcgencmd get_throttled | cut -d= -f2)
VAL=$((RAW))
echo "raw: ${RAW}"

check_bit() {
  local bit=$1
  local label=$2
  if (( (VAL >> bit) & 1 )); then
    echo "  [!]  ${label}"
  else
    echo "  [OK] ${label}"
  fi
}

check_bit 0  "電圧不足(現在)"
check_bit 1  "ARM周波数制限(現在)"
check_bit 2  "スロットリング中(現在)"
check_bit 3  "温度制限作動中(現在)"
check_bit 16 "電圧不足(起動後に発生あり)"
check_bit 17 "ARM周波数制限(起動後に発生あり)"
check_bit 18 "スロットリング(起動後に発生あり)"
check_bit 19 "温度制限(起動後に発生あり)"

echo ""
echo "=== 温度 ==="
vcgencmd measure_temp

echo ""
echo "=== 電圧 (core) ==="
vcgencmd measure_volts core

echo ""
echo "=== メモリ割当 ==="
vcgencmd get_mem arm
vcgencmd get_mem gpu

echo ""
echo "=== ARMクロック ==="
HZ=$(vcgencmd measure_clock arm | cut -d= -f2)
echo "$(( HZ / 1000000 )) MHz (raw: ${HZ} Hz)"
