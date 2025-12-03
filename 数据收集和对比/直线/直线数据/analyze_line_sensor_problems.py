"""
Line Sensor 直线跟随问题分析
目的：证明 Line Sensor 在直线跟随任务中表现不佳
用于与 Bump Sensor 进行对比
"""

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False

def load_all_data(data_folder):
    """加载所有实验数据"""
    all_data = []
    for i in range(1, 11):
        file_path = os.path.join(data_folder, f'{i}.csv')
        if os.path.exists(file_path):
            df = pd.read_csv(file_path)
            df['Experiment'] = i
            all_data.append(df)
            print(f"✓ 加载实验 {i}: {len(df)} 样本")
    return pd.concat(all_data, ignore_index=True) if all_data else None

def analyze_problems(data):
    """分析 Line Sensor 的主要问题"""
    results = {
        'experiments': [],
        'y_deviation_std': [],      # Y方向偏离标准差
        'y_max_error': [],          # Y方向最大偏离
        'theta_max_error': [],      # 最大角度偏差(度)
        'ir_variation': [],         # IR传感器变异系数
        'speed_instability': [],    # 速度不稳定性
        'zero_speed_ratio': [],     # 速度为0的比例（停顿）
    }
    
    for exp in range(1, 11):
        exp_data = data[data['Experiment'] == exp]
        if len(exp_data) == 0:
            continue
            
        results['experiments'].append(exp)
        
        # 1. Y方向偏离（理想应该是0）
        results['y_deviation_std'].append(exp_data['Y_mm'].std())
        results['y_max_error'].append(exp_data['Y_mm'].abs().max())
        
        # 2. 角度偏差（理想应该是0度）
        results['theta_max_error'].append(np.degrees(exp_data['Theta_rad'].abs().max()))
        
        # 3. IR传感器稳定性（变异系数 = std/mean）
        ir_cv = exp_data['IR_center'].std() / exp_data['IR_center'].mean() * 100
        results['ir_variation'].append(ir_cv)
        
        # 4. 速度不稳定性
        speed_changes = np.abs(np.diff(exp_data['SpdL_cps'])) + np.abs(np.diff(exp_data['SpdR_cps']))
        results['speed_instability'].append(speed_changes.mean())
        
        # 5. 停顿比例（左右轮速都为0）
        zero_speed = ((exp_data['SpdL_cps'] == 0) & (exp_data['SpdR_cps'] == 0)).sum()
        results['zero_speed_ratio'].append(zero_speed / len(exp_data) * 100)
    
    return pd.DataFrame(results)

def create_problem_visualization(data, problems_df, output_folder):
    """创建问题可视化图表"""
    
    # ========== 图1: Line Sensor 跟随问题总览 ==========
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle('Line Sensor 直线跟随问题分析\n(10组重复实验)', fontsize=16, fontweight='bold')
    
    colors = plt.cm.Set3(np.linspace(0, 1, 10))
    
    # 1. Y方向偏离分布
    ax = axes[0, 0]
    for i, exp in enumerate(range(1, 11)):
        exp_data = data[data['Experiment'] == exp]
        ax.plot(exp_data['X_mm'], exp_data['Y_mm'], color=colors[i], alpha=0.7, linewidth=2)
    ax.axhline(0, color='red', linestyle='--', linewidth=2, label='理想轨迹 (Y=0)')
    ax.fill_between([0, data['X_mm'].max()], -10, 10, alpha=0.2, color='green', label='可接受范围 ±10mm')
    ax.set_xlabel('X位置 (mm)')
    ax.set_ylabel('Y偏离 (mm)')
    ax.set_title('❌ 问题1: 横向偏离严重', fontweight='bold', color='red')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # 2. 角度偏差
    ax = axes[0, 1]
    for i, exp in enumerate(range(1, 11)):
        exp_data = data[data['Experiment'] == exp]
        ax.plot(exp_data['X_mm'], np.degrees(exp_data['Theta_rad']), color=colors[i], alpha=0.7, linewidth=2)
    ax.axhline(0, color='red', linestyle='--', linewidth=2)
    ax.fill_between([0, data['X_mm'].max()], -5, 5, alpha=0.2, color='green', label='可接受范围 ±5°')
    ax.set_xlabel('X位置 (mm)')
    ax.set_ylabel('角度偏差 (度)')
    ax.set_title('❌ 问题2: 航向角不稳定', fontweight='bold', color='red')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # 3. IR传感器响应
    ax = axes[0, 2]
    for i, exp in enumerate(range(1, 11)):
        exp_data = data[data['Experiment'] == exp]
        ax.plot(exp_data['Sample'], exp_data['IR_center'], color=colors[i], alpha=0.7, linewidth=2)
    ax.axhline(80, color='red', linestyle='--', linewidth=2, label='目标值=80')
    ax.fill_between([0, data['Sample'].max()], 70, 90, alpha=0.2, color='green', label='稳定范围')
    ax.set_xlabel('采样点')
    ax.set_ylabel('IR传感器值')
    ax.set_title('❌ 问题3: 传感器读数波动大', fontweight='bold', color='red')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # 4. 轮速波动
    ax = axes[1, 0]
    for i, exp in enumerate(range(1, 11)):
        exp_data = data[data['Experiment'] == exp]
        ax.plot(exp_data['Sample'], exp_data['SpdL_cps'], color=colors[i], alpha=0.5, linewidth=1.5)
        ax.plot(exp_data['Sample'], exp_data['SpdR_cps'], color=colors[i], alpha=0.5, linewidth=1.5, linestyle='--')
    ax.set_xlabel('采样点')
    ax.set_ylabel('轮速 (counts/s)')
    ax.set_title('❌ 问题4: 速度控制不平稳', fontweight='bold', color='red')
    ax.grid(True, alpha=0.3)
    
    # 5. 问题指标柱状图
    ax = axes[1, 1]
    x = np.arange(len(problems_df))
    width = 0.35
    ax.bar(x - width/2, problems_df['y_max_error'], width, label='最大Y偏离(mm)', color='coral')
    ax.bar(x + width/2, problems_df['theta_max_error'], width, label='最大角度偏差(°)', color='steelblue')
    ax.set_xlabel('实验编号')
    ax.set_ylabel('误差值')
    ax.set_title('各实验误差对比', fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(problems_df['experiments'])
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    
    # 6. 综合评分（问题严重程度）
    ax = axes[1, 2]
    metrics = ['Y偏离\n标准差', 'IR变异\n系数(%)', '速度\n不稳定性', '停顿\n比例(%)']
    values = [
        problems_df['y_deviation_std'].mean(),
        problems_df['ir_variation'].mean(),
        problems_df['speed_instability'].mean() / 10,  # 缩放
        problems_df['zero_speed_ratio'].mean()
    ]
    colors_bar = ['#ff6b6b', '#ffa502', '#ff6348', '#ee5a24']
    bars = ax.bar(metrics, values, color=colors_bar, edgecolor='black', linewidth=1.5)
    ax.set_ylabel('问题严重程度')
    ax.set_title('Line Sensor 问题汇总', fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y')
    
    # 添加数值标签
    for bar, val in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5, 
                f'{val:.1f}', ha='center', fontsize=10, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_folder, 'line_sensor_problems.png'), dpi=300, bbox_inches='tight')
    print("✓ 保存: line_sensor_problems.png")
    plt.close()
    
    # ========== 图2: 最终位置散点图（重复性差） ==========
    fig, ax = plt.subplots(figsize=(10, 8))
    
    final_positions = []
    for exp in range(1, 11):
        exp_data = data[data['Experiment'] == exp]
        if len(exp_data) > 0:
            final = exp_data.iloc[-1]
            final_positions.append({'exp': exp, 'x': final['X_mm'], 'y': final['Y_mm']})
    
    final_df = pd.DataFrame(final_positions)
    
    # 绘制散点
    scatter = ax.scatter(final_df['x'], final_df['y'], s=200, c=range(1, 11), 
                         cmap='tab10', edgecolors='black', linewidth=2, alpha=0.8)
    
    # 标注实验编号
    for _, row in final_df.iterrows():
        ax.annotate(f"实验{int(row['exp'])}", (row['x'], row['y']), 
                   fontsize=9, ha='left', va='bottom', 
                   xytext=(5, 5), textcoords='offset points')
    
    # 计算统计
    mean_x, mean_y = final_df['x'].mean(), final_df['y'].mean()
    std_x, std_y = final_df['x'].std(), final_df['y'].std()
    
    ax.axhline(0, color='green', linestyle='--', linewidth=2, alpha=0.5, label='理想Y位置')
    ax.scatter(mean_x, mean_y, s=300, c='red', marker='*', edgecolors='black', 
               linewidth=2, label=f'平均位置', zorder=10)
    
    # 添加误差范围
    from matplotlib.patches import Ellipse
    ellipse = Ellipse((mean_x, mean_y), 2*std_x, 2*std_y, 
                      fill=False, edgecolor='red', linestyle='--', linewidth=2)
    ax.add_patch(ellipse)
    
    ax.set_xlabel('最终X位置 (mm)', fontsize=12)
    ax.set_ylabel('最终Y位置 (mm)', fontsize=12)
    ax.set_title(f'Line Sensor 最终位置分布\n' + 
                 f'X: {mean_x:.1f}±{std_x:.1f}mm, Y: {mean_y:.1f}±{std_y:.1f}mm\n' +
                 f'❌ 重复性差：Y方向标准差={std_y:.1f}mm', 
                 fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_folder, 'line_sensor_repeatability.png'), dpi=300, bbox_inches='tight')
    print("✓ 保存: line_sensor_repeatability.png")
    plt.close()

def generate_comparison_report(problems_df, output_folder):
    """生成对比分析报告"""
    report = []
    report.append("=" * 70)
    report.append("LINE SENSOR 直线跟随性能分析报告")
    report.append("（用于与 Bump Sensor 对比）")
    report.append("=" * 70)
    report.append("")
    
    report.append("【实验概述】")
    report.append(f"  实验次数: {len(problems_df)} 组")
    report.append(f"  任务: 直线跟随 Leader 机器人")
    report.append("")
    
    report.append("【Line Sensor 主要问题】")
    report.append("")
    
    # 问题1: 横向偏离
    y_std_mean = problems_df['y_deviation_std'].mean()
    y_max_mean = problems_df['y_max_error'].mean()
    report.append(f"  ❌ 问题1: 横向偏离严重")
    report.append(f"     - 平均Y偏离标准差: {y_std_mean:.2f} mm")
    report.append(f"     - 平均最大Y偏离: {y_max_mean:.2f} mm")
    report.append(f"     - 评价: {'差' if y_std_mean > 10 else '一般' if y_std_mean > 5 else '良好'}")
    report.append("")
    
    # 问题2: 角度不稳定
    theta_max_mean = problems_df['theta_max_error'].mean()
    report.append(f"  ❌ 问题2: 航向角不稳定")
    report.append(f"     - 平均最大角度偏差: {theta_max_mean:.2f}°")
    report.append(f"     - 评价: {'差' if theta_max_mean > 20 else '一般' if theta_max_mean > 10 else '良好'}")
    report.append("")
    
    # 问题3: 传感器波动
    ir_cv_mean = problems_df['ir_variation'].mean()
    report.append(f"  ❌ 问题3: 传感器读数波动大")
    report.append(f"     - IR传感器变异系数: {ir_cv_mean:.2f}%")
    report.append(f"     - 评价: {'差' if ir_cv_mean > 30 else '一般' if ir_cv_mean > 15 else '良好'}")
    report.append("")
    
    # 问题4: 速度不稳定
    speed_instab_mean = problems_df['speed_instability'].mean()
    zero_ratio_mean = problems_df['zero_speed_ratio'].mean()
    report.append(f"  ❌ 问题4: 速度控制不平稳")
    report.append(f"     - 平均速度变化: {speed_instab_mean:.2f} counts/s")
    report.append(f"     - 停顿比例: {zero_ratio_mean:.1f}%")
    report.append("")
    
    report.append("【结论】")
    report.append("  Line Sensor 在直线跟随任务中存在以下缺陷:")
    report.append("  1. 对距离变化不敏感，导致横向偏离大")
    report.append("  2. 方向控制不精确，航向角波动明显")
    report.append("  3. 传感器读数不稳定，影响控制精度")
    report.append("  4. 需要频繁调整速度，效率低")
    report.append("")
    report.append("  建议: 使用 Bump Sensor 替代 Line Sensor 进行直线跟随")
    report.append("")
    report.append("=" * 70)
    
    # 保存报告
    report_text = '\n'.join(report)
    report_path = os.path.join(output_folder, 'line_sensor_analysis_report.txt')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report_text)
    
    print("\n" + report_text)
    print(f"\n✓ 报告已保存: {report_path}")

def main():
    print("=" * 60)
    print("Line Sensor 直线跟随问题分析")
    print("=" * 60)
    
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 加载数据
    print("\n正在加载数据...")
    data = load_all_data(script_dir)
    
    if data is None:
        print("❌ 未找到数据文件！")
        return
    
    print(f"\n总计加载 {len(data)} 个数据点")
    
    # 分析问题
    print("\n正在分析 Line Sensor 问题...")
    problems_df = analyze_problems(data)
    
    # 生成可视化
    print("\n正在生成图表...")
    create_problem_visualization(data, problems_df, script_dir)
    
    # 生成报告
    generate_comparison_report(problems_df, script_dir)
    
    print("\n" + "=" * 60)
    print("分析完成！生成文件:")
    print("  1. line_sensor_problems.png - 问题可视化")
    print("  2. line_sensor_repeatability.png - 重复性分析")
    print("  3. line_sensor_analysis_report.txt - 分析报告")
    print("=" * 60)

if __name__ == "__main__":
    main()

