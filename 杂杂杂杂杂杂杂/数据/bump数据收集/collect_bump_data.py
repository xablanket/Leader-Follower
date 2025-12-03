#!/usr/bin/env python3
"""
碰撞传感器自动化数据收集脚本

功能:
1. 自动连接Arduino串口
2. 批量收集多个距离的传感器数据
3. 自动整合成一个CSV文件
4. 支持断点续传

使用方法:
1. 上传 Bump_only.ino 到Arduino
2. 运行: python collect_bump_data.py
3. 根据提示设置好距离，然后按Arduino上的按钮开始采集
"""

import serial
import serial.tools.list_ports
import time
import sys
from pathlib import Path
import pandas as pd
from datetime import datetime

# ============================================
# 配置
# ============================================
# 默认要测试的距离列表（cm）
DEFAULT_DISTANCES = [3, 5, 7, 10, 12, 15, 20, 25, 30, 35, 40]

# 串口配置
BAUD_RATE = 115200
TIMEOUT = 2

# 输出文件（在脚本所在目录创建）
SCRIPT_DIR = Path(__file__).parent
OUTPUT_DIR = SCRIPT_DIR / 'bump_sensor_data'
TIMESTAMP = datetime.now().strftime('%Y%m%d_%H%M%S')

# ============================================
# 串口操作
# ============================================
def find_arduino_port():
    """
    自动查找Arduino串口
    """
    ports = serial.tools.list_ports.comports()
    
    print("\n🔍 正在搜索Arduino...")
    print("可用串口:")
    
    for i, port in enumerate(ports):
        print(f"  [{i}] {port.device} - {port.description}")
    
    if not ports:
        print("❌ 错误: 未找到任何串口设备")
        print("   请检查Arduino是否已连接")
        sys.exit(1)
    
    # 如果只有一个串口，自动选择
    if len(ports) == 1:
        return ports[0].device
    
    # 尝试自动识别Arduino
    for port in ports:
        if 'Arduino' in port.description or 'CH340' in port.description or 'USB' in port.description:
            print(f"✅ 自动识别到Arduino: {port.device}")
            return port.device
    
    # 手动选择
    while True:
        try:
            choice = input(f"\n请选择串口 [0-{len(ports)-1}]: ")
            idx = int(choice)
            if 0 <= idx < len(ports):
                return ports[idx].device
        except (ValueError, KeyboardInterrupt):
            print("\n❌ 取消操作")
            sys.exit(0)

def connect_serial(port, max_retries=3):
    """
    连接到Arduino串口（带重试机制）
    """
    for attempt in range(max_retries):
        try:
            if attempt > 0:
                print(f"\n🔄 重试连接... (尝试 {attempt + 1}/{max_retries})")
            
            ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT)
            time.sleep(2)  # 等待Arduino复位
            
            # 清空缓冲区
            ser.flushInput()
            ser.flushOutput()
            
            print(f"✅ 已连接到 {port} (波特率: {BAUD_RATE})")
            return ser
            
        except serial.SerialException as e:
            if "PermissionError" in str(e) or "Access is denied" in str(e) or "拒绝访问" in str(e):
                print(f"❌ 串口被占用: {port}")
                print(f"   可能原因:")
                print(f"   - Arduino IDE的串口监视器还在运行")
                print(f"   - 其他程序正在使用该串口")
                print(f"   - 需要管理员权限")
                
                if attempt < max_retries - 1:
                    print(f"\n⏳ 请关闭占用串口的程序，然后...")
                    input(f"   按Enter继续重试，或按Ctrl+C取消")
                else:
                    print(f"\n💡 解决方法:")
                    print(f"   1. 关闭Arduino IDE的串口监视器")
                    print(f"   2. 关闭其他串口程序（PuTTY/CoolTerm等）")
                    print(f"   3. 重新插拔Arduino USB线")
                    print(f"   4. 以管理员身份运行此脚本")
                    sys.exit(1)
            else:
                print(f"❌ 连接失败: {e}")
                if attempt < max_retries - 1:
                    time.sleep(1)
                else:
                    sys.exit(1)
        
        except Exception as e:
            print(f"❌ 未知错误: {e}")
            if attempt >= max_retries - 1:
                sys.exit(1)
            time.sleep(1)

# ============================================
# 数据采集
# ============================================
def send_distance(ser, distance):
    """
    发送距离值到Arduino
    """
    ser.write(f"{distance}\n".encode())
    ser.flush()
    time.sleep(0.1)
    
    # 读取确认信息
    while ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(f"   📡 Arduino回应: {line}")
            if "[Distance Set To]" in line:
                return True
    return True

def collect_data_for_distance(ser, distance):
    """
    采集单个距离的数据
    """
    print(f"\n{'='*60}")
    print(f"📏 准备采集距离: {distance} cm")
    print(f"{'='*60}")
    
    # 发送距离
    print(f"   ⏳ 正在设置距离...")
    send_distance(ser, distance)
    
    # 等待用户按按钮
    print(f"   ⚠️  请将机器人放置在距离 {distance} cm 的位置")
    print(f"   ⚠️  然后按Arduino上的按钮开始采集...")
    
    data_lines = []
    recording = False
    
    while True:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if not line:
                continue
            
            print(f"   📥 {line}")
            
            # 检测开始标记
            if "DATA RECORD START" in line:
                recording = True
                print(f"   ✅ 开始采集数据...")
                continue
            
            # 检测结束标记
            if "DATA RECORD END" in line:
                print(f"   ✅ 数据采集完成！共 {len(data_lines)} 条")
                break
            
            # 保存数据行（跳过CSV表头）
            if recording and line and not line.startswith("distance_cm"):
                if ',' in line:  # 确保是CSV数据行
                    data_lines.append(line)
    
    return data_lines

def save_data(all_data, filename):
    """
    保存所有数据到CSV文件
    """
    # 解析CSV数据
    rows = []
    for line in all_data:
        parts = line.split(',')
        if len(parts) >= 5:  # distance_cm,sample_id,bump_L,bump_R,bump_avg
            rows.append({
                'distance_cm': int(parts[0]),
                'sample_id': int(parts[1]),
                'bump_L': int(parts[2]),
                'bump_R': int(parts[3]),
                'bump_avg': int(parts[4])
            })
    
    # 创建DataFrame
    df = pd.DataFrame(rows)
    
    # 保存到CSV
    df.to_csv(filename, index=False)
    print(f"\n✅ 数据已保存到: {filename}")
    print(f"   总数据点: {len(df)}")
    
    return df

def print_statistics(df):
    """
    打印数据统计信息
    """
    print(f"\n{'='*60}")
    print("📊 数据统计")
    print(f"{'='*60}")
    
    for distance in df['distance_cm'].unique():
        subset = df[df['distance_cm'] == distance]
        print(f"\n距离 {distance} cm:")
        print(f"  数据点数:       {len(subset)}")
        print(f"  bump_L 平均:    {subset['bump_L'].mean():.1f} (±{subset['bump_L'].std():.1f})")
        print(f"  bump_R 平均:    {subset['bump_R'].mean():.1f} (±{subset['bump_R'].std():.1f})")
        print(f"  bump_avg 平均:  {subset['bump_avg'].mean():.1f} (±{subset['bump_avg'].std():.1f})")
        print(f"  bump_avg 范围:  [{subset['bump_avg'].min()}, {subset['bump_avg'].max()}]")
    
    print(f"\n{'='*60}")

# ============================================
# 主函数
# ============================================
def main():
    print("\n" + "="*60)
    print("  碰撞传感器自动化数据收集工具")
    print("="*60)
    
    # 创建输出目录
    OUTPUT_DIR.mkdir(exist_ok=True)
    
    # 配置距离列表
    print(f"\n默认距离列表: {DEFAULT_DISTANCES}")
    choice = input("使用默认距离? [Y/n]: ").strip().lower()
    
    if choice == 'n':
        print("请输入距离列表（用逗号分隔，例如: 3,5,10,15,20）")
        distances_input = input("距离列表: ")
        try:
            distances = [int(d.strip()) for d in distances_input.split(',')]
        except ValueError:
            print("❌ 输入格式错误，使用默认距离列表")
            distances = DEFAULT_DISTANCES
    else:
        distances = DEFAULT_DISTANCES
    
    print(f"\n✅ 将采集以下距离的数据: {distances}")
    print(f"   总共需要采集: {len(distances)} 组数据")
    
    # 连接Arduino
    port = find_arduino_port()
    ser = connect_serial(port)
    
    # 等待Arduino就绪
    print("\n⏳ 等待Arduino初始化...")
    time.sleep(2)
    
    # 清空初始输出
    while ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(f"   {line}")
    
    # 开始采集
    print(f"\n{'='*60}")
    print("🚀 开始数据采集")
    print(f"{'='*60}")
    
    all_data = []
    completed_distances = []
    
    try:
        for i, distance in enumerate(distances, 1):
            print(f"\n进度: [{i}/{len(distances)}]")
            
            data_lines = collect_data_for_distance(ser, distance)
            all_data.extend(data_lines)
            completed_distances.append(distance)
            
            # 中间保存（防止数据丢失）
            if i % 3 == 0 or i == len(distances):
                temp_file = OUTPUT_DIR / f"bump_data_temp_{TIMESTAMP}.csv"
                save_data(all_data, temp_file)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断采集")
        print(f"   已完成距离: {completed_distances}")
    
    finally:
        ser.close()
        print("\n🔌 串口已关闭")
    
    # 保存最终数据
    if all_data:
        final_file = OUTPUT_DIR / f"bump_data_{TIMESTAMP}.csv"
        df = save_data(all_data, final_file)
        
        # 打印统计
        print_statistics(df)
        
        print(f"\n{'='*60}")
        print("✅ 数据采集完成！")
        print(f"   数据文件: {final_file}")
        print(f"   已完成距离: {completed_distances}")
        print(f"{'='*60}\n")
    else:
        print("\n❌ 未采集到任何数据")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 再见！")
        sys.exit(0)

