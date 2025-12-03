#!/usr/bin/env python3
"""
Line Sensor 数据分析脚本

功能:
1. 读取line sensor的CSV数据
2. 绘制距离-读数关系图
3. 计算校准曲线
4. 生成传感器性能报告
5. 与bump sensor性能对比

使用方法:
1. 直接运行: python analyze_line_data.py (自动查找最新文件)
2. 指定文件: python analyze_line_data.py line_sensor_data/line_data_xxx.csv
"""

import sys
import pandas as pd
import numpy as np
from pathlib import Path
from scipy.optimize import curve_fit

# 设置matplotlib使用非交互式后端（解决Tk错误）
import matplotlib
matplotlib.use('Agg')  # 必须在import pyplot之前设置
import matplotlib.pyplot as plt

# 设置中文字体（可选）
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial']
plt.rcParams['axes.unicode_minus'] = False

# ============================================
# 配置
# ============================================
FIGURE_SIZE = (12, 8)
DPI = 100
LINE_WIDTH = 2
MARKER_SIZE = 8

# ============================================
# 数据读取
# ============================================
def load_data(filename):
    """
    读取CSV数据文件
    
    期望格式: distance_cm,sample_id,line_1,line_2,line_3,center_avg
    (只使用中间3个传感器，去掉最左line_0和最右line_4)
    """
    try:
        data = pd.read_csv(filename)
        print(f"✅ 成功读取数据: {filename}")
        print(f"   数据点数: {len(data)}")
        print(f"   列名: {list(data.columns)}")
        
        # 检查必需的列
        required_cols = ['distance_cm', 'center_avg']
        missing = [col for col in required_cols if col not in data.columns]
        if missing:
            print(f"❌ 错误: 缺少必需的列: {missing}")
            sys.exit(1)
        
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
    计算每个距离的统计信息
    """
    stats = {}
    
    for distance in sorted(data['distance_cm'].unique()):
        subset = data[data['distance_cm'] == distance]
        
        # 计算中间3个传感器统计 (line_1, line_2, line_3)
        line_stats = {}
        for i in [1, 2, 3]:  # 只有中间3个
            col_name = f'line_{i}'
            if col_name in subset.columns:
                line_stats[f'line_{i}_mean'] = subset[col_name].mean()
                line_stats[f'line_{i}_std'] = subset[col_name].std()
        
        stats[distance] = {
            'count': len(subset),
            'center_avg_mean': subset['center_avg'].mean(),
            'center_avg_std': subset['center_avg'].std(),
            'center_avg_min': subset['center_avg'].min(),
            'center_avg_max': subset['center_avg'].max(),
            'cv': (subset['center_avg'].std() / subset['center_avg'].mean() * 100) if subset['center_avg'].mean() > 0 else 0,
            **line_stats
        }
    
    return stats

def print_statistics(stats):
    """
    打印统计结果
    """
    print("\n" + "="*70)
    print("📊 Line Sensor 统计数据 (只使用中间3个传感器)")
    print("="*70)
    print(f"{'距离(cm)':<10} {'样本数':<8} {'中间3个平均值':<20} {'CV%':<8}")
    print("-"*70)
    
    for distance in sorted(stats.keys()):
        s = stats[distance]
        center_str = f"{s['center_avg_mean']:7.1f}±{s['center_avg_std']:5.1f}"
        print(f"{distance:<10} {s['count']:<8} {center_str:<20} {s['cv']:6.2f}")
    
    print("="*70 + "\n")
    print("注: CV% = 变异系数 (标准差/平均值×100)，越小越稳定")
    print("    CV < 5%: 优秀  |  5-10%: 良好  |  10-20%: 一般  |  >20%: 较差")
    print()

# ============================================
# 校准曲线拟合
# ============================================
def power_law(x, a, b):
    """幂律模型: y = a * x^b"""
    return a * np.power(x, b)

def inverse_model(x, a, b):
    """反比模型: y = a / (x + b)"""
    return a / (x + b)

def fit_calibration_curve(distances, readings):
    """
    拟合校准曲线
    """
    # 过滤掉无效数据
    valid_mask = (readings > 0) & (distances > 0)
    x = distances[valid_mask]
    y = readings[valid_mask]
    
    if len(x) < 3:
        print("⚠️  警告: 有效数据点太少，无法拟合曲线")
        return None, None, None
    
    try:
        # 尝试幂律拟合
        popt_power, _ = curve_fit(power_law, x, y, p0=[100, -1], maxfev=5000)
        y_pred_power = power_law(x, *popt_power)
        r2_power = 1 - (np.sum((y - y_pred_power)**2) / np.sum((y - np.mean(y))**2))
        
        # 尝试反比拟合
        popt_inv, _ = curve_fit(inverse_model, x, y, p0=[1000, 1], maxfev=5000)
        y_pred_inv = inverse_model(x, *popt_inv)
        r2_inv = 1 - (np.sum((y - y_pred_inv)**2) / np.sum((y - np.mean(y))**2))
        
        # 选择拟合效果更好的模型
        if r2_power > r2_inv:
            return 'power', popt_power, r2_power
        else:
            return 'inverse', popt_inv, r2_inv
            
    except Exception as e:
        print(f"⚠️  警告: 曲线拟合失败: {e}")
        return None, None, None

# ============================================
# 数据可视化
# ============================================
def plot_raw_data(data, output_file='line_raw_data.png'):
    """
    绘制原始数据散点图（只有中间3个传感器）
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # 子图1: 中间3个传感器 (line_1, line_2, line_3)
    colors = ['blue', 'green', 'red']
    for i, sensor_idx in enumerate([1, 2, 3]):
        col = f'line_{sensor_idx}'
        if col in data.columns:
            axes[0, 0].scatter(data['distance_cm'], data[col], 
                             alpha=0.5, s=20, color=colors[i], label=f'Sensor {sensor_idx}')
    axes[0, 0].set_xlabel('Distance (cm)', fontsize=12)
    axes[0, 0].set_ylabel('IR Signal Strength', fontsize=12)
    axes[0, 0].set_title('Center 3 Sensors (Line 1, 2, 3)', fontsize=14, fontweight='bold')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()
    
    # 子图2: 各传感器对比
    for i, sensor_idx in enumerate([1, 2, 3]):
        col = f'line_{sensor_idx}'
        if col in data.columns:
            axes[0, 1].scatter(data['distance_cm'], data[col], 
                             alpha=0.4, s=15, color=colors[i], label=f'Sensor {sensor_idx}')
    axes[0, 1].set_xlabel('Distance (cm)', fontsize=12)
    axes[0, 1].set_ylabel('IR Signal Strength', fontsize=12)
    axes[0, 1].set_title('Individual Sensor Readings', fontsize=14, fontweight='bold')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()
    
    # 子图3: Center Average (用于控制)
    axes[1, 0].scatter(data['distance_cm'], data['center_avg'], 
                      alpha=0.5, s=20, color='purple')
    axes[1, 0].set_xlabel('Distance (cm)', fontsize=12)
    axes[1, 0].set_ylabel('Center Average', fontsize=12)
    axes[1, 0].set_title('Center Average (Control Signal)', fontsize=14, fontweight='bold')
    axes[1, 0].grid(True, alpha=0.3)
    
    # 子图4: 分布密度
    axes[1, 1].hist(data['center_avg'], bins=30, alpha=0.7, color='purple', edgecolor='black')
    axes[1, 1].set_xlabel('Center Average', fontsize=12)
    axes[1, 1].set_ylabel('Frequency', fontsize=12)
    axes[1, 1].set_title('Distribution of Center Average', fontsize=14, fontweight='bold')
    axes[1, 1].grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=DPI)
    print(f"✅ 原始数据图已保存: {output_file}")

def plot_calibration_curve(data, stats, output_file='line_calibration.png'):
    """
    绘制校准曲线（带误差棒）
    """
    distances = []
    means = []
    stds = []
    
    for distance in sorted(stats.keys()):
        distances.append(distance)
        means.append(stats[distance]['center_avg_mean'])
        stds.append(stats[distance]['center_avg_std'])
    
    distances = np.array(distances)
    means = np.array(means)
    stds = np.array(stds)
    
    # 拟合曲线
    model_type, params, r2 = fit_calibration_curve(distances, means)
    
    plt.figure(figsize=FIGURE_SIZE)
    
    # 绘制原始数据点（散点）
    plt.scatter(data['distance_cm'], data['center_avg'], 
               alpha=0.2, s=10, color='gray', label='Raw Data')
    
    # 绘制平均值和误差棒
    plt.errorbar(distances, means, yerr=stds, 
                fmt='o', markersize=MARKER_SIZE, capsize=5, capthick=2,
                color='purple', ecolor='lavender', 
                label='Mean ± Std Dev', linewidth=LINE_WIDTH)
    
    # 绘制拟合曲线
    if model_type and params is not None:
        x_fit = np.linspace(distances.min(), distances.max(), 200)
        if model_type == 'power':
            y_fit = power_law(x_fit, *params)
            equation = f'y = {params[0]:.2f} × x^{params[1]:.3f}'
        else:
            y_fit = inverse_model(x_fit, *params)
            equation = f'y = {params[0]:.2f} / (x + {params[1]:.3f})'
        
        plt.plot(x_fit, y_fit, 'r-', linewidth=LINE_WIDTH+1, 
                label=f'Fit: {equation}\nR² = {r2:.4f}')
    
    plt.xlabel('Distance (cm)', fontsize=14, fontweight='bold')
    plt.ylabel('Line Sensor IR Reading (Center 3 avg)', fontsize=14, fontweight='bold')
    plt.title('Line Sensor Calibration Curve', fontsize=16, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=11)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=DPI)
    print(f"✅ 校准曲线图已保存: {output_file}")
    
    return model_type, params, r2

def plot_sensor_consistency(data, stats, output_file='line_consistency.png'):
    """
    绘制传感器一致性分析图（只有中间3个）
    """
    fig, axes = plt.subplots(2, 1, figsize=(12, 10))
    
    distances = sorted(stats.keys())
    colors_sensor = ['blue', 'green', 'red']
    
    # 子图1: 中间3个传感器对比
    for idx, i in enumerate([1, 2, 3]):  # 中间3个传感器
        means = [stats[d].get(f'line_{i}_mean', 0) for d in distances]
        axes[0].plot(distances, means, 'o-', linewidth=LINE_WIDTH, 
                    markersize=MARKER_SIZE, color=colors_sensor[idx], label=f'Sensor {i}')
    
    axes[0].set_xlabel('Distance (cm)', fontsize=12)
    axes[0].set_ylabel('Reading', fontsize=12)
    axes[0].set_title('Center 3 Sensors Consistency', fontsize=14, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    
    # 子图2: 变异系数 (CV)
    cvs = [stats[d]['cv'] for d in distances]
    colors = ['green' if cv < 5 else 'orange' if cv < 10 else 'red' for cv in cvs]
    
    axes[1].bar(distances, cvs, color=colors, alpha=0.7, edgecolor='black')
    axes[1].axhline(y=5, color='green', linestyle='--', linewidth=2, label='Excellent (CV < 5%)')
    axes[1].axhline(y=10, color='orange', linestyle='--', linewidth=2, label='Good (CV < 10%)')
    axes[1].set_xlabel('Distance (cm)', fontsize=12)
    axes[1].set_ylabel('Coefficient of Variation (%)', fontsize=12)
    axes[1].set_title('Measurement Stability (CV%)', fontsize=14, fontweight='bold')
    axes[1].grid(True, alpha=0.3, axis='y')
    axes[1].legend()
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=DPI)
    print(f"✅ 一致性分析图已保存: {output_file}")

def plot_distance_ranges(stats, output_file='line_ranges.png'):
    """
    绘制不同距离的读数范围
    """
    distances = sorted(stats.keys())
    means = [stats[d]['center_avg_mean'] for d in distances]
    mins = [stats[d]['center_avg_min'] for d in distances]
    maxs = [stats[d]['center_avg_max'] for d in distances]
    
    plt.figure(figsize=FIGURE_SIZE)
    
    # 绘制范围区域
    plt.fill_between(distances, mins, maxs, alpha=0.3, color='purple', label='Min-Max Range')
    
    # 绘制平均值线
    plt.plot(distances, means, 'o-', linewidth=LINE_WIDTH+1, 
            markersize=MARKER_SIZE+2, color='red', label='Mean')
    
    plt.xlabel('Distance (cm)', fontsize=14, fontweight='bold')
    plt.ylabel('Line Sensor Reading', fontsize=14, fontweight='bold')
    plt.title('Reading Range by Distance', fontsize=16, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=12)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=DPI)
    print(f"✅ 读数范围图已保存: {output_file}")

def plot_temporal_stability(data, output_file='line_temporal.png'):
    """
    绘制时间序列稳定性图
    """
    distances = sorted(data['distance_cm'].unique())
    
    # 创建颜色映射
    colors = plt.cm.tab10(np.linspace(0, 1, len(distances)))
    
    plt.figure(figsize=(14, 8))
    
    for i, distance in enumerate(distances):
        subset = data[data['distance_cm'] == distance].sort_values('sample_id')
        
        # 绘制center_avg线
        plt.plot(subset['sample_id'], subset['center_avg'], 
                'o-', linewidth=LINE_WIDTH, markersize=4,
                color=colors[i], label=f'{distance} cm', alpha=0.8)
    
    plt.xlabel('Sample ID (时间序列)', fontsize=14, fontweight='bold')
    plt.ylabel('Center Average Reading', fontsize=14, fontweight='bold')
    plt.title('Measurement Stability Over Time for Different Distances', 
             fontsize=16, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(title='Distance', fontsize=10, ncol=2)
    
    # 添加说明文本
    plt.text(0.02, 0.98, 
            '水平线 = 稳定性好\n上下波动 = 不稳定', 
            transform=plt.gca().transAxes,
            fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=DPI)
    print(f"✅ 时间序列稳定性图已保存: {output_file}")

# ============================================
# 生成报告
# ============================================
def generate_report(data, stats, model_type, params, r2, output_file='line_report.txt'):
    """
    生成详细分析报告
    """
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write("="*70 + "\n")
        f.write("  Line Sensor Performance Report\n")
        f.write("  (Using center 3 line sensors as IR receivers)\n")
        f.write("  Strategy: Focus on straight-line distance measurement\n")
        f.write("="*70 + "\n\n")
        
        f.write("数据概览:\n")
        f.write(f"  总数据点数: {len(data)}\n")
        f.write(f"  测试距离数: {len(stats)}\n")
        f.write(f"  距离范围: {min(stats.keys())}-{max(stats.keys())} cm\n")
        f.write(f"  读数范围: {data['center_avg'].min():.0f}-{data['center_avg'].max():.0f}\n")
        f.write("\n")
        
        f.write("="*70 + "\n")
        f.write("校准结果:\n")
        f.write("="*70 + "\n\n")
        
        if model_type and params is not None:
            f.write(f"拟合模型: {model_type.upper()}\n")
            if model_type == 'power':
                f.write(f"  公式: reading = {params[0]:.4f} × distance^{params[1]:.4f}\n")
            else:
                f.write(f"  公式: reading = {params[0]:.4f} / (distance + {params[1]:.4f})\n")
            f.write(f"  拟合优度: R² = {r2:.4f}\n")
            
            if r2 > 0.95:
                f.write("  评价: 优秀拟合 ⭐⭐⭐⭐⭐\n")
            elif r2 > 0.90:
                f.write("  评价: 良好拟合 ⭐⭐⭐⭐\n")
            elif r2 > 0.80:
                f.write("  评价: 一般拟合 ⭐⭐⭐\n")
            else:
                f.write("  评价: 较差拟合 ⭐⭐\n")
        else:
            f.write("  拟合失败或数据不足\n")
        
        f.write("\n" + "="*70 + "\n")
        f.write("传感器性能评估:\n")
        f.write("="*70 + "\n\n")
        
        # 计算整体性能指标
        avg_cv = np.mean([stats[d]['cv'] for d in stats.keys()])
        max_cv = np.max([stats[d]['cv'] for d in stats.keys()])
        
        f.write(f"1. 稳定性 (变异系数CV):\n")
        f.write(f"   平均CV: {avg_cv:.2f}%\n")
        f.write(f"   最大CV: {max_cv:.2f}%\n")
        if avg_cv < 5:
            f.write("   评价: 优秀 ⭐⭐⭐⭐⭐\n")
        elif avg_cv < 10:
            f.write("   评价: 良好 ⭐⭐⭐⭐\n")
        elif avg_cv < 20:
            f.write("   评价: 一般 ⭐⭐⭐\n")
        else:
            f.write("   评价: 较差 ⭐⭐\n")
        
        f.write("\n" + "="*70 + "\n")
        f.write("与Bump Sensor对比:\n")
        f.write("="*70 + "\n\n")
        f.write("Line Sensor 特点:\n")
        f.write("  优势:\n")
        f.write("    - 原本设计用于线路追踪，硬件成熟\n")
        f.write("    - 5个传感器提供冗余测量\n")
        f.write("    - 角度检测能力（通过左右传感器差异）\n")
        f.write("  劣势:\n")
        f.write("    - 非专用距离传感器，精度可能较低\n")
        f.write("    - 受环境光影响较大\n")
        f.write("    - 读数范围和灵敏度未优化用于距离测量\n\n")
        
        f.write("="*70 + "\n")
        f.write("建议:\n")
        f.write("="*70 + "\n\n")
        
        if r2 and r2 > 0.90 and avg_cv < 10:
            f.write("✅ Line sensor性能可接受，可用于距离测量\n")
        elif r2 and r2 > 0.80:
            f.write("⚠️  Line sensor性能一般，建议:\n")
            f.write("   - 优化Leader的IR发射强度\n")
            f.write("   - 改善测量环境（减少环境光干扰）\n")
        else:
            f.write("❌ Line sensor作为距离传感器性能不佳\n")
            f.write("   - 建议使用专用的bump/distance sensor\n")
        
        f.write("\n" + "="*70 + "\n")
    
    print(f"✅ 分析报告已保存: {output_file}")

# ============================================
# 主函数
# ============================================
def main():
    print("\n" + "="*70)
    print("  Line Sensor Data Analysis Tool")
    print("="*70 + "\n")
    
    # 检查命令行参数
    if len(sys.argv) < 2:
        # 自动查找最新文件 - 支持多个可能的目录
        script_dir = Path(__file__).parent  # 脚本所在目录
        possible_dirs = [
            script_dir / 'line_sensor_data',  # 同目录下
            Path('line_sensor_data'),  # 当前工作目录
            Path('数据收集和对比/line sensor/line_sensor_data'),  # 完整路径
        ]
        
        data_dir = None
        for dir_path in possible_dirs:
            if dir_path.exists():
                data_dir = dir_path
                break
        
        if data_dir and data_dir.exists():
            csv_files = sorted(data_dir.glob('line_data_*.csv'), 
                             key=lambda x: x.stat().st_mtime, reverse=True)
            # 排除temp文件
            csv_files = [f for f in csv_files if 'temp' not in f.name]
            if csv_files:
                filename = str(csv_files[0])
                print(f"📁 自动使用最新的CSV文件:")
                print(f"   {filename}\n")
            else:
                print("❌ line_sensor_data/ 目录中没有找到CSV文件")
                print("\n使用方法: python analyze_line_data.py <csv_file>")
                sys.exit(1)
        else:
            print("❌ 未找到 line_sensor_data/ 目录")
            print("\n已尝试查找以下位置:")
            for dir_path in possible_dirs:
                print(f"   - {dir_path}")
            print("\n使用方法: python analyze_line_data.py <csv_file>")
            sys.exit(1)
    else:
        filename = sys.argv[1]
    
    # 创建输出目录（在脚本所在目录）
    script_dir = Path(__file__).parent
    output_dir = script_dir / 'line_analysis_results'
    output_dir.mkdir(exist_ok=True)
    print(f"📁 输出目录: {output_dir}/\n")
    
    # 读取数据
    data = load_data(filename)
    
    # 计算统计
    stats = calculate_statistics(data)
    print_statistics(stats)
    
    # 生成图表
    print("📊 正在生成图表...\n")
    plot_raw_data(data, output_dir / 'line_raw_data.png')
    model_type, params, r2 = plot_calibration_curve(data, stats, output_dir / 'line_calibration.png')
    plot_sensor_consistency(data, stats, output_dir / 'line_consistency.png')
    plot_distance_ranges(stats, output_dir / 'line_ranges.png')
    plot_temporal_stability(data, output_dir / 'line_temporal.png')
    
    # 生成报告
    print("\n📝 正在生成报告...\n")
    generate_report(data, stats, model_type, params, r2, output_dir / 'line_report.txt')
    
    # 显示拟合结果
    if model_type and params is not None:
        print("\n" + "="*70)
        print("📐 校准公式:")
        print("="*70)
        if model_type == 'power':
            print(f"  reading = {params[0]:.4f} × distance^{params[1]:.4f}")
        else:
            print(f"  reading = {params[0]:.4f} / (distance + {params[1]:.4f})")
        print(f"  R² = {r2:.4f}")
        print("="*70)
    
    print("\n" + "="*70)
    print("✅ 分析完成！")
    print(f"   所有文件已保存到: {output_dir}/")
    print("="*70 + "\n")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 分析中断")
        sys.exit(0)

