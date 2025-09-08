#!/bin/bash
# CAN接口配置脚本
# 配置can0和can1为RH2双手控制

echo "🔧 配置RH2双手CAN接口..."
echo "================================"

# 检查是否以root权限运行
if [ "$EUID" -ne 0 ]; then
    echo "❌ 请以root权限运行此脚本:"
    echo "   sudo $0"
    exit 1
fi

# 设置CAN接口
echo "📡 配置CAN接口..."

# 停止接口（如果已启动）
echo "⏹️  停止现有CAN接口..."
ip link set down can0 2>/dev/null || true
ip link set down can1 2>/dev/null || true

# 配置can0 (右手)
echo "🤚 配置can0 (右手) - 波特率1Mbps..."
if ip link set can0 type can bitrate 1000000; then
    echo "✅ can0配置成功"
else
    echo "❌ can0配置失败"
    exit 1
fi

# 配置can1 (左手)  
echo "🤚 配置can1 (左手) - 波特率1Mbps..."
if ip link set can1 type can bitrate 1000000; then
    echo "✅ can1配置成功"
else
    echo "❌ can1配置失败"
    exit 1
fi

# 启动接口
echo "🚀 启动CAN接口..."
if ip link set up can0; then
    echo "✅ can0启动成功"
else
    echo "❌ can0启动失败"
    exit 1
fi

if ip link set up can1; then
    echo "✅ can1启动成功"
else
    echo "❌ can1启动失败"
    exit 1
fi

# 显示状态
echo ""
echo "📊 CAN接口状态:"
ip link show | grep can

echo ""
echo "✨ CAN接口配置完成!"
echo "📋 配置信息:"
echo "   can0: 右手控制 (1Mbps)"
echo "   can1: 左手控制 (1Mbps)"
echo ""
echo "💡 现在可以运行RH2硬件测试:"
echo "   python3 run_hardware_test.py"
