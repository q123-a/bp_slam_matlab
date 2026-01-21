#!/bin/bash
# 运行 MATLAB 诊断脚本

cd /home/qlb/slam/n/bp_slam/BP_SLAM-main

# 如果有 octave，尝试用 octave
if command -v octave &> /dev/null; then
    echo "使用 Octave 运行诊断..."
    octave --no-gui --eval "diagnose_dataVA"
else
    echo "Octave 未安装，请在 MATLAB 中手动运行: diagnose_dataVA"
fi
