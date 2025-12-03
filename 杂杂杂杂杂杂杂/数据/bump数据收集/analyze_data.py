#!/usr/bin/env python3
"""
Hypothesis 3 - 实验数据分析脚本

功能:
1. 读取Serial Monitor导出的CSV数据
2. 绘制各项指标的时间序列图
3. 计算性能统计指标
4. 生成实验报告

使用方法:
1. 从Serial Monitor复制数据
2. 保存为CSV文件 (例如: follower_data.csv)
3. 运行: python analyze_data.py follower_data.csv
"""

import sys
import pandas as pd
import numpy as np
from pathlib import Path

# 设置matplotlib使用非交互式后端（解决Tk错误）
import matplotlib
matplotlib.use('Agg')  # 必须在import pyplot之前设置
import matplotlib.pyplot as plt

# ============================================
# 配置
# ============================================
FIGURE_SIZE = (12, 8)
DPI = 100
LINE_WIDTH = 2

# ============================================
# 数据读取
# ============================================
def load_data(filename):
    """
    读取CSV数据文件
    
    期望的CSV格式 (从Serial Monitor输出):
    Time,Bump,LineErr,Base,Turn,X,Y
    """
    try:
        data = pd.read_csv(filename)
        print(f"✅ 成功读取数据: {filename}")
        print(f"   数据点数: {len(data)}")
        print(f"   列名: {list(data.columns)}")
        return data
    except FileNotFoundError:
        print(f"❌ 错误: 找不到文件 {filename}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ 错误: {e}")
        sys.exit(1)

# ============================================
# 数据分析
# ============================================
def calculate_statistics(data):
    """
    计算关键性能指标
    """
    stats = {}
    
    # Bump信号统计
    if 'Bump' in data.columns:
        stats['bump_mean'] = data['Bump'].mean()
        stats['bump_std'] = data['Bump'].std()
        stats['bump_min'] = data['Bump'].min()
        stats['bump_max'] = data['Bump'].max()
    
    # Line偏差统计
    if 'LineErr' in data.columns:
        stats['line_mean'] = data['LineErr'].mean()
        stats['line_std'] = data['LineErr'].std()
        stats['line_max_abs'] = data['LineErr'].abs().max()
    
    # 速度统计
    if 'Base' in data.columns:
        stats['base_mean'] = data['Base'].mean()
        stats['base_std'] = data['Base'].std()
    
    if 'Turn' in data.columns:
        stats['turn_mean'] = data['Turn'].mean()
        stats['turn_std'] = data['Turn'].std()
        stats['turn_max_abs'] = data['Turn'].abs().max()
    
    # 位置统计
    if 'X' in data.columns and 'Y' in data.columns:
        # 计算总行程
        dx = data['X'].diff()
        dy = data['Y'].diff()
        distance = np.sqrt(dx**2 + dy**2)
        stats['total_distance'] = distance.sum()
        
        # 最终位置
        stats['final_x'] = data['X'].iloc[-1]
        stats['final_y'] = data['Y'].iloc[-1]
    
    return stats

def print_statistics(stats):
    """
    打印统计结果
    """
    print("\n" + "="*50)
    print("📊 实验数据统计")
    print("="*50)
    
    if 'bump_mean' in stats:
        print("\n🔵 Bump传感器 (距离控制)")
        print(f"  平均信号强度: {stats['bump_mean']:.2f}")
        print(f"  标准差:       {stats['bump_std']:.2f}")
        print(f"  范围:         [{stats['bump_min']:.2f}, {stats['bump_max']:.2f}]")
    
    if 'line_mean' in stats:
        print("\n🔴 Line传感器 (方向控制)")
        print(f"  平均偏差:     {stats['line_mean']:.3f}")
        print(f"  标准差:       {stats['line_std']:.3f}")
        print(f"  最大偏差:     {stats['line_max_abs']:.3f}")
    
    if 'base_mean' in stats:
        print("\n⚡ 速度控制")
        print(f"  平均基础速度: {stats['base_mean']:.2f}")
        print(f"  速度标准差:   {stats['base_std']:.2f}")
    
    if 'turn_mean' in stats:
        print(f"  平均转向量:   {stats['turn_mean']:.2f}")
        print(f"  转向标准差:   {stats['turn_std']:.2f}")
        print(f"  最大转向量:   {stats['turn_max_abs']:.2f}")
    
    if 'total_distance' in stats:
        print("\n📍 运动轨迹")
        print(f"  总行程:       {stats['total_distance']:.2f} mm")
        print(f"  最终位置:     X={stats['final_x']:.2f}, Y={stats['final_y']:.2f} mm")
    
    print("="*50 + "\n")

# ============================================
# 数据可视化
# ============================================
def plot_signals(data, output_file='signals.png'):
    """
    绘制传感器信号图
    """
    fig, axes = plt.subplots(2, 1, figsize=FIGURE_SIZE)
    
    # Bump信号
    if 'Bump' in data.columns:
        axes[0].plot(data['Time'], data['Bump'], 
                    linewidth=LINE_WIDTH, color='blue', label='Bump Signal')
        axes[0].set_ylabel('Bump Signal Strength', fontsize=12)
        axes[0].set_title('Bump Sensor (Distance Control)', fontsize=14, fontweight='bold')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
    
    # Line偏差
    if 'LineErr' in data.columns:
        axes[1].plot(data['Time'], data['LineErr'], 
                    linewidth=LINE_WIDTH, color='red', label='Line Error')
        axes[1].axhline(y=0, color='black', linestyle='--', alpha=0.3)
        axes[1].set_ylabel('Line Error', fontsize=12)
        axes[1].set_xlabel('Time (s)', fontsize=12)
        axes[1].set_title('Line Sensor (Direction Control)', fontsize=14, fontweight='bold')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=DPI)
    print(f"✅ 传感器信号图已保存: {output_file}")

def plot_control(data, output_file='control.png'):
    """
    绘制控制输出图
    """
    fig, axes = plt.subplots(2, 1, figsize=FIGURE_SIZE)
    
    # 基础速度
    if 'Base' in data.columns:
        axes[0].plot(data['Time'], data['Base'], 
                    linewidth=LINE_WIDTH, color='green', label='Base Speed')
        axes[0].set_ylabel('Base Speed (PWM)', fontsize=12)
        axes[0].set_title('Base Speed (from Bump PID)', fontsize=14, fontweight='bold')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
    
    # 转向输出
    if 'Turn' in data.columns:
        axes[1].plot(data['Time'], data['Turn'], 
                    linewidth=LINE_WIDTH, color='orange', label='Turn Output')
        axes[1].axhline(y=0, color='black', linestyle='--', alpha=0.3)
        axes[1].set_ylabel('Turn Output (PWM)', fontsize=12)
        axes[1].set_xlabel('Time (s)', fontsize=12)
        axes[1].set_title('Turn Output (from Line PID)', fontsize=14, fontweight='bold')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=DPI)
    print(f"✅ 控制输出图已保存: {output_file}")

def plot_trajectory(data, output_file='trajectory.png'):
    """
    绘制运动轨迹图
    """
    if 'X' not in data.columns or 'Y' not in data.columns:
        print("⚠️  警告: 数据中没有X/Y坐标，跳过轨迹图")
        return
    
    plt.figure(figsize=(10, 10))
    
    # 绘制轨迹
    plt.plot(data['X'], data['Y'], linewidth=LINE_WIDTH, color='purple', 
            marker='o', markersize=3, alpha=0.7, label='Trajectory')
    
    # 标记起点和终点
    plt.plot(data['X'].iloc[0], data['Y'].iloc[0], 
            'go', markersize=15, label='Start', zorder=5)
    plt.plot(data['X'].iloc[-1], data['Y'].iloc[-1], 
            'ro', markersize=15, label='End', zorder=5)
    
    plt.xlabel('X (mm)', fontsize=12)
    plt.ylabel('Y (mm)', fontsize=12)
    plt.title('Follower Trajectory', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.legend(fontsize=12)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=DPI)
    print(f"✅ 运动轨迹图已保存: {output_file}")

def plot_pwm(data, output_file='pwm.png'):
    """
    绘制最终PWM输出图
    """
    # 计算PWM (如果有Base和Turn)
    if 'Base' in data.columns and 'Turn' in data.columns:
        left_pwm = data['Base'] - data['Turn']
        right_pwm = data['Base'] + data['Turn']
        
        plt.figure(figsize=FIGURE_SIZE)
        
        plt.plot(data['Time'], left_pwm, 
                linewidth=LINE_WIDTH, color='blue', label='Left PWM')
        plt.plot(data['Time'], right_pwm, 
                linewidth=LINE_WIDTH, color='red', label='Right PWM')
        
        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel('PWM Value', fontsize=12)
        plt.title('Final Motor PWM Outputs', fontsize=14, fontweight='bold')
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=12)
        
        plt.tight_layout()
        plt.savefig(output_file, dpi=DPI)
        print(f"✅ PWM输出图已保存: {output_file}")
    else:
        print("⚠️  警告: 数据中没有Base/Turn，跳过PWM图")

def plot_combined(data, output_file='combined.png'):
    """
    绘制综合分析图 (4合1)
    """
    fig = plt.figure(figsize=(16, 12))
    
    # 子图1: Bump信号
    ax1 = plt.subplot(2, 2, 1)
    if 'Bump' in data.columns:
        ax1.plot(data['Time'], data['Bump'], linewidth=LINE_WIDTH, color='blue')
        ax1.set_ylabel('Bump Signal')
        ax1.set_title('Bump Sensor (Distance)', fontweight='bold')
        ax1.grid(True, alpha=0.3)
    
    # 子图2: Line偏差
    ax2 = plt.subplot(2, 2, 2)
    if 'LineErr' in data.columns:
        ax2.plot(data['Time'], data['LineErr'], linewidth=LINE_WIDTH, color='red')
        ax2.axhline(y=0, color='black', linestyle='--', alpha=0.3)
        ax2.set_ylabel('Line Error')
        ax2.set_title('Line Sensor (Direction)', fontweight='bold')
        ax2.grid(True, alpha=0.3)
    
    # 子图3: 控制输出
    ax3 = plt.subplot(2, 2, 3)
    if 'Base' in data.columns:
        ax3.plot(data['Time'], data['Base'], linewidth=LINE_WIDTH, 
                color='green', label='Base Speed')
    if 'Turn' in data.columns:
        ax3.plot(data['Time'], data['Turn'], linewidth=LINE_WIDTH, 
                color='orange', label='Turn Output')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Control Output')
    ax3.set_title('PID Outputs', fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # 子图4: 轨迹
    ax4 = plt.subplot(2, 2, 4)
    if 'X' in data.columns and 'Y' in data.columns:
        ax4.plot(data['X'], data['Y'], linewidth=LINE_WIDTH, color='purple')
        ax4.plot(data['X'].iloc[0], data['Y'].iloc[0], 'go', markersize=10)
        ax4.plot(data['X'].iloc[-1], data['Y'].iloc[-1], 'ro', markersize=10)
        ax4.set_xlabel('X (mm)')
        ax4.set_ylabel('Y (mm)')
        ax4.set_title('Trajectory', fontweight='bold')
        ax4.grid(True, alpha=0.3)
        ax4.axis('equal')
    
    plt.suptitle('Hypothesis 3 - Experimental Results', 
                fontsize=16, fontweight='bold', y=0.995)
    plt.tight_layout()
    plt.savefig(output_file, dpi=DPI)
    print(f"✅ 综合分析图已保存: {output_file}")

# ============================================
# 生成报告
# ============================================
def generate_report(data, stats, output_file='report.txt'):
    """
    生成文本格式的实验报告
    """
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write("="*60 + "\n")
        f.write("  Hypothesis 3 - Leader-Follower实验报告\n")
        f.write("="*60 + "\n\n")
        
        f.write("实验信息:\n")
        f.write(f"  数据点数: {len(data)}\n")
        if 'Time' in data.columns:
            f.write(f"  实验时长: {data['Time'].max():.2f} 秒\n")
        f.write("\n")
        
        f.write("="*60 + "\n")
        f.write("性能统计\n")
        f.write("="*60 + "\n\n")
        
        if 'bump_mean' in stats:
            f.write("1. Bump传感器 (距离控制)\n")
            f.write(f"   - 平均信号强度: {stats['bump_mean']:.2f}\n")
            f.write(f"   - 标准差:       {stats['bump_std']:.2f}\n")
            f.write(f"   - 范围:         [{stats['bump_min']:.2f}, {stats['bump_max']:.2f}]\n")
            f.write(f"   - 稳定性评价:   ")
            if stats['bump_std'] < 20:
                f.write("优秀 ⭐⭐⭐⭐⭐\n")
            elif stats['bump_std'] < 40:
                f.write("良好 ⭐⭐⭐⭐\n")
            elif stats['bump_std'] < 60:
                f.write("一般 ⭐⭐⭐\n")
            else:
                f.write("较差 ⭐⭐\n")
            f.write("\n")
        
        if 'line_mean' in stats:
            f.write("2. Line传感器 (方向控制)\n")
            f.write(f"   - 平均偏差:     {stats['line_mean']:.3f}\n")
            f.write(f"   - 标准差:       {stats['line_std']:.3f}\n")
            f.write(f"   - 最大偏差:     {stats['line_max_abs']:.3f}\n")
            f.write(f"   - 对中性评价:   ")
            if abs(stats['line_mean']) < 0.3:
                f.write("优秀 ⭐⭐⭐⭐⭐\n")
            elif abs(stats['line_mean']) < 0.6:
                f.write("良好 ⭐⭐⭐⭐\n")
            elif abs(stats['line_mean']) < 1.0:
                f.write("一般 ⭐⭐⭐\n")
            else:
                f.write("较差 ⭐⭐\n")
            f.write("\n")
        
        if 'base_mean' in stats:
            f.write("3. 速度控制\n")
            f.write(f"   - 平均基础速度: {stats['base_mean']:.2f}\n")
            f.write(f"   - 速度标准差:   {stats['base_std']:.2f}\n")
            if 'turn_mean' in stats:
                f.write(f"   - 平均转向量:   {stats['turn_mean']:.2f}\n")
                f.write(f"   - 转向标准差:   {stats['turn_std']:.2f}\n")
                f.write(f"   - 最大转向量:   {stats['turn_max_abs']:.2f}\n")
            f.write("\n")
        
        if 'total_distance' in stats:
            f.write("4. 运动轨迹\n")
            f.write(f"   - 总行程:       {stats['total_distance']:.2f} mm\n")
            f.write(f"   - 最终位置:     X={stats['final_x']:.2f}, Y={stats['final_y']:.2f} mm\n")
            f.write("\n")
        
        f.write("="*60 + "\n")
        f.write("总体评价\n")
        f.write("="*60 + "\n\n")
        
        # 简单的总体评分
        score = 0
        max_score = 0
        
        if 'bump_std' in stats:
            max_score += 1
            if stats['bump_std'] < 30:
                score += 1
        
        if 'line_std' in stats:
            max_score += 1
            if stats['line_std'] < 0.5:
                score += 1
        
        if max_score > 0:
            percentage = (score / max_score) * 100
            f.write(f"系统稳定性得分: {score}/{max_score} ({percentage:.0f}%)\n\n")
            
            if percentage >= 80:
                f.write("✅ 系统表现优秀！跟随稳定，控制精确。\n")
            elif percentage >= 60:
                f.write("✅ 系统表现良好，建议微调PID参数。\n")
            else:
                f.write("⚠️  系统需要改进，建议检查传感器和PID参数。\n")
        
        f.write("\n" + "="*60 + "\n")
    
    print(f"✅ 实验报告已保存: {output_file}")

# ============================================
# 主函数
# ============================================
def main():
    print("\n" + "="*60)
    print("  Hypothesis 3 - 实验数据分析工具")
    print("="*60 + "\n")
    
    # 检查命令行参数
    if len(sys.argv) < 2:
        # 如果没有命令行参数，尝试自动查找最新的CSV文件
        data_dir = Path('bump_sensor_data')
        if data_dir.exists():
            csv_files = sorted(data_dir.glob('bump_data_*.csv'), key=lambda x: x.stat().st_mtime, reverse=True)
            if csv_files:
                filename = str(csv_files[0])
                print(f"📁 未指定文件，自动使用最新的CSV文件:")
                print(f"   {filename}\n")
            else:
                print("❌ bump_sensor_data/ 目录中没有找到CSV文件")
                print("\n使用方法: python analyze_data.py <csv_file>")
                print("示例:     python analyze_data.py bump_sensor_data/bump_data_20251126_215830.csv")
                sys.exit(1)
        else:
            print("❌ 未找到 bump_sensor_data/ 目录")
            print("\n使用方法: python analyze_data.py <csv_file>")
            print("示例:     python analyze_data.py follower_data.csv")
            sys.exit(1)
    else:
        filename = sys.argv[1]
    
    # 创建输出目录
    output_dir = Path('analysis_results')
    output_dir.mkdir(exist_ok=True)
    print(f"📁 输出目录: {output_dir}/\n")
    
    # 读取数据
    data = load_data(filename)
    
    # 计算统计
    stats = calculate_statistics(data)
    print_statistics(stats)
    
    # 生成图表
    print("📊 正在生成图表...\n")
    plot_signals(data, output_dir / 'signals.png')
    plot_control(data, output_dir / 'control.png')
    plot_trajectory(data, output_dir / 'trajectory.png')
    plot_pwm(data, output_dir / 'pwm.png')
    plot_combined(data, output_dir / 'combined.png')
    
    # 生成报告
    print("\n📝 正在生成报告...\n")
    generate_report(data, stats, output_dir / 'report.txt')
    
    print("\n" + "="*60)
    print("✅ 分析完成！")
    print(f"   所有文件已保存到: {output_dir}/")
    print("="*60 + "\n")

if __name__ == '__main__':
    main()

