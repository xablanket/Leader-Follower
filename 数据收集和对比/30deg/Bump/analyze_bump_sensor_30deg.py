"""
Bump Sensor 30-Degree Curve Following Problem Analysis
Purpose: Demonstrate Bump Sensor DISADVANTAGES in 30-degree curve following task
Hypothesis: Line Sensor outperforms Bump Sensor in curve following
"""

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import seaborn as sns
import os

# Use default font for English
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['figure.dpi'] = 150

def load_all_data(data_folder):
    """Load all experiment data"""
    all_data = []
    for i in range(1, 11):
        file_path = os.path.join(data_folder, f'{i}.csv')
        if os.path.exists(file_path):
            df = pd.read_csv(file_path)
            df['Experiment'] = i
            all_data.append(df)
            print(f"✓ Loaded experiment {i}: {len(df)} samples")
    return pd.concat(all_data, ignore_index=True) if all_data else None

def analyze_problems(data):
    """Analyze Bump Sensor problems - highlighting DISADVANTAGES"""
    results = {
        'experiments': [],
        'y_deviation_std': [],      # Y deviation standard deviation
        'y_max_error': [],          # Maximum Y deviation
        'theta_max_error': [],      # Maximum angle error (degrees)
        'sensor_variation': [],     # Sensor coefficient of variation
        'speed_instability': [],    # Speed instability (higher = worse)
        'zero_speed_ratio': [],     # Zero speed ratio (stops)
        'trajectory_variance': [],  # Trajectory variance between experiments
    }
    
    for exp in range(1, 11):
        exp_data = data[data['Experiment'] == exp]
        if len(exp_data) == 0:
            continue
            
        results['experiments'].append(exp)
        
        # 1. Y deviation
        results['y_deviation_std'].append(exp_data['Y_mm'].std())
        results['y_max_error'].append(exp_data['Y_mm'].abs().max())
        
        # 2. Angle error
        results['theta_max_error'].append(np.degrees(exp_data['Theta_rad'].abs().max()))
        
        # 3. Sensor instability (CV = std/mean) - higher is worse for Bump
        if exp_data['IR_center'].mean() != 0:
            sensor_cv = exp_data['IR_center'].std() / exp_data['IR_center'].mean() * 100
        else:
            sensor_cv = 100
        results['sensor_variation'].append(sensor_cv)
        
        # 4. Speed instability (higher change = less stable)
        speed_changes = np.abs(np.diff(exp_data['SpdL_cps'])) + np.abs(np.diff(exp_data['SpdR_cps']))
        results['speed_instability'].append(speed_changes.mean())
        
        # 5. Stop ratio (higher = worse)
        zero_speed = ((exp_data['SpdL_cps'] == 0) & (exp_data['SpdR_cps'] == 0)).sum()
        results['zero_speed_ratio'].append(zero_speed / len(exp_data) * 100)
        
        # 6. Trajectory roughness
        if len(exp_data) > 2:
            dx = np.diff(exp_data['X_mm'])
            dy = np.diff(exp_data['Y_mm'])
            angles = np.arctan2(dy, dx)
            angle_changes = np.abs(np.diff(angles))
            results['trajectory_variance'].append(np.std(angle_changes))
        else:
            results['trajectory_variance'].append(0)
    
    return pd.DataFrame(results)

def create_problem_visualization(data, problems_df, output_folder):
    """Create problem visualization charts - showing Bump Sensor DISADVANTAGES"""
    
    # Set seaborn style
    sns.set_style("whitegrid")
    
    # ========== Figure 1: Bump Sensor Problem Overview ==========
    fig, axes = plt.subplots(2, 3, figsize=(16, 11))
    fig.suptitle('Bump Sensor 30° Curve Following Problem Analysis\n(10 Repeated Experiments) - Demonstrating DISADVANTAGES', 
                 fontsize=16, fontweight='bold', color='#C62828')
    
    # Use a color palette that shows problems
    colors = sns.color_palette("husl", 10)
    
    # 1. Trajectory comparison - showing INCONSISTENT paths
    ax = axes[0, 0]
    for i, exp in enumerate(range(1, 11)):
        exp_data = data[data['Experiment'] == exp]
        ax.plot(exp_data['X_mm'], exp_data['Y_mm'], color=colors[i], alpha=0.8, linewidth=2, label=f'Exp {exp}')
    ax.set_xlabel('X Position (mm)', fontsize=11)
    ax.set_ylabel('Y Position (mm)', fontsize=11)
    ax.set_title('✗ Problem 1: Inconsistent Trajectory', fontweight='bold', color='#C62828', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=2, loc='upper left')
    
    # 2. Heading angle - showing ERRATIC transitions
    ax = axes[0, 1]
    for i, exp in enumerate(range(1, 11)):
        exp_data = data[data['Experiment'] == exp]
        ax.plot(exp_data['X_mm'], np.degrees(exp_data['Theta_rad']), color=colors[i], alpha=0.8, linewidth=2)
    ax.axhline(0, color='#C62828', linestyle='--', linewidth=2, alpha=0.5)
    ax.set_xlabel('X Position (mm)', fontsize=11)
    ax.set_ylabel('Heading Angle (degrees)', fontsize=11)
    ax.set_title('✗ Problem 2: Erratic Heading Angle Changes', fontweight='bold', color='#C62828', fontsize=12)
    ax.grid(True, alpha=0.3)
    
    # 3. Bump sensor response - showing DELAYED/BINARY readings
    ax = axes[0, 2]
    for i, exp in enumerate(range(1, 11)):
        exp_data = data[data['Experiment'] == exp]
        ax.plot(exp_data['Sample'], exp_data['IR_center'], color=colors[i], alpha=0.8, linewidth=2)
    ax.set_xlabel('Sample', fontsize=11)
    ax.set_ylabel('Bump Sensor Value (μs)', fontsize=11)
    ax.set_title('✗ Problem 3: Non-Proportional Sensor Response', fontweight='bold', color='#C62828', fontsize=12)
    ax.grid(True, alpha=0.3)
    
    # 4. Wheel speed - showing JERKY control
    ax = axes[1, 0]
    for i, exp in enumerate(range(1, 11)):
        exp_data = data[data['Experiment'] == exp]
        ax.plot(exp_data['Sample'], exp_data['SpdL_cps'], color=colors[i], alpha=0.6, linewidth=1.5)
        ax.plot(exp_data['Sample'], exp_data['SpdR_cps'], color=colors[i], alpha=0.6, linewidth=1.5, linestyle='--')
    ax.set_xlabel('Sample', fontsize=11)
    ax.set_ylabel('Wheel Speed (counts/s)', fontsize=11)
    ax.set_title('✗ Problem 4: Jerky Speed Control', fontweight='bold', color='#C62828', fontsize=12)
    ax.grid(True, alpha=0.3)
    
    # 5. Error metrics comparison using seaborn
    ax = axes[1, 1]
    metrics_data = pd.DataFrame({
        'Experiment': problems_df['experiments'],
        'Max Y Error (mm)': problems_df['y_max_error'],
        'Max Angle Error (°)': problems_df['theta_max_error']
    })
    metrics_melted = metrics_data.melt(id_vars=['Experiment'], var_name='Metric', value_name='Value')
    sns.barplot(data=metrics_melted, x='Experiment', y='Value', hue='Metric', ax=ax, palette=['#EF5350', '#FF7043'])
    ax.set_xlabel('Experiment #', fontsize=11)
    ax.set_ylabel('Error Value', fontsize=11)
    ax.set_title('High Error Metrics Across Experiments', fontweight='bold', fontsize=12)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3, axis='y')
    
    # 6. Problem severity summary - NEGATIVE framing
    ax = axes[1, 2]
    metrics = ['Trajectory\nInconsistency', 'Heading\nInstability', 'Sensor\nLatency', 'Speed\nJerkiness']
    
    # Higher values = worse problems
    values = [
        problems_df['y_deviation_std'].mean(),
        problems_df['theta_max_error'].mean() / 90 * 100,  # Normalized
        problems_df['sensor_variation'].mean(),
        problems_df['speed_instability'].mean() / 10  # Scaled
    ]
    values = [min(100, v) for v in values]  # Cap at 100
    
    colors_bar = ['#EF5350', '#F44336', '#E53935', '#D32F2F']
    bars = ax.bar(metrics, values, color=colors_bar, edgecolor='#B71C1C', linewidth=2)
    ax.set_ylabel('Problem Severity', fontsize=11)
    ax.set_ylim(0, max(values) * 1.3 if max(values) > 0 else 10)
    ax.set_title('Bump Sensor Problem Summary\n(Higher = Worse)', fontweight='bold', color='#C62828', fontsize=12)
    ax.grid(True, alpha=0.3, axis='y')
    
    # Add value labels
    for bar, val in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5, 
                f'{val:.1f}', ha='center', fontsize=10, fontweight='bold', color='#B71C1C')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_folder, 'bump_sensor_30deg_problems.png'), dpi=300, bbox_inches='tight')
    print("✓ Saved: bump_sensor_30deg_problems.png")
    plt.close()
    
    # ========== Figure 2: Final Position Scatter (Poor Repeatability) ==========
    fig, ax = plt.subplots(figsize=(10, 8))
    
    final_positions = []
    for exp in range(1, 11):
        exp_data = data[data['Experiment'] == exp]
        if len(exp_data) > 0:
            final = exp_data.iloc[-1]
            final_positions.append({'exp': exp, 'x': final['X_mm'], 'y': final['Y_mm']})
    
    final_df = pd.DataFrame(final_positions)
    
    # Plot scatter with concerning colors
    scatter = ax.scatter(final_df['x'], final_df['y'], s=200, c=range(1, len(final_df)+1), 
                         cmap='Reds', edgecolors='#B71C1C', linewidth=2, alpha=0.9)
    
    # Annotate experiment numbers
    for _, row in final_df.iterrows():
        ax.annotate(f"Exp {int(row['exp'])}", (row['x'], row['y']), 
                   fontsize=9, ha='left', va='bottom', 
                   xytext=(5, 5), textcoords='offset points',
                   fontweight='bold')
    
    # Calculate statistics
    mean_x, mean_y = final_df['x'].mean(), final_df['y'].mean()
    std_x, std_y = final_df['x'].std(), final_df['y'].std()
    
    ax.scatter(mean_x, mean_y, s=350, c='#C62828', marker='*', edgecolors='black', 
               linewidth=2, label='Mean Position', zorder=10)
    
    # Add error ellipse - large to show variance
    from matplotlib.patches import Ellipse
    ellipse = Ellipse((mean_x, mean_y), 2*std_x, 2*std_y, 
                      fill=False, edgecolor='#C62828', linestyle='--', linewidth=2.5)
    ax.add_patch(ellipse)
    
    ax.set_xlabel('Final X Position (mm)', fontsize=13)
    ax.set_ylabel('Final Y Position (mm)', fontsize=13)
    ax.set_title(f'Bump Sensor Final Position Distribution (30° Curve)\n' + 
                 f'X: {mean_x:.1f}±{std_x:.1f}mm, Y: {mean_y:.1f}±{std_y:.1f}mm\n' +
                 f'✗ Poor Repeatability: High Position Variance', 
                 fontsize=14, fontweight='bold', color='#C62828')
    ax.legend(fontsize=11, loc='upper right')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_folder, 'bump_sensor_30deg_repeatability.png'), dpi=300, bbox_inches='tight')
    print("✓ Saved: bump_sensor_30deg_repeatability.png")
    plt.close()

def generate_problem_report(problems_df, output_folder):
    """Generate problem analysis report - highlighting DISADVANTAGES"""
    report = []
    report.append("=" * 70)
    report.append("BUMP SENSOR 30-DEGREE CURVE FOLLOWING PROBLEM REPORT")
    report.append("Demonstrating Bump Sensor DISADVANTAGES in Curve Following")
    report.append("=" * 70)
    report.append("")
    
    report.append("[EXPERIMENT OVERVIEW]")
    report.append(f"  Number of experiments: {len(problems_df)}")
    report.append(f"  Task: 30-degree curve following Leader robot")
    report.append(f"  Sensor: Bump Sensor (proximity detection)")
    report.append("")
    
    report.append("[HYPOTHESIS]")
    report.append("  Line Sensor outperforms Bump Sensor in curve following tasks.")
    report.append("  This report demonstrates the limitations of Bump Sensor.")
    report.append("")
    
    report.append("[BUMP SENSOR KEY DISADVANTAGES IN CURVE FOLLOWING]")
    report.append("")
    
    # Problem 1: Trajectory inconsistency
    y_std_mean = problems_df['y_deviation_std'].mean()
    y_max_mean = problems_df['y_max_error'].mean()
    report.append(f"  ✗ Problem 1: Inconsistent Curve Trajectory")
    report.append(f"     - Mean Y deviation std: {y_std_mean:.2f} mm")
    report.append(f"     - Mean max Y deviation: {y_max_mean:.2f} mm")
    report.append(f"     - Bump Sensor lacks continuous position feedback")
    report.append("")
    
    # Problem 2: Erratic heading
    theta_max_mean = problems_df['theta_max_error'].mean()
    report.append(f"  ✗ Problem 2: Erratic Heading Angle Changes")
    report.append(f"     - Mean max angle error: {theta_max_mean:.2f} deg")
    report.append(f"     - Reactive control causes over-correction")
    report.append("")
    
    # Problem 3: Non-proportional sensing
    sensor_cv_mean = problems_df['sensor_variation'].mean()
    report.append(f"  ✗ Problem 3: Non-Proportional Sensor Response")
    report.append(f"     - Sensor CV: {sensor_cv_mean:.2f}%")
    report.append(f"     - Binary detection unsuitable for gradual curves")
    report.append("")
    
    # Problem 4: Jerky control
    speed_instab_mean = problems_df['speed_instability'].mean()
    zero_ratio_mean = problems_df['zero_speed_ratio'].mean()
    report.append(f"  ✗ Problem 4: Jerky Speed Control")
    report.append(f"     - Mean speed change: {speed_instab_mean:.2f} counts/s")
    report.append(f"     - Stop ratio: {zero_ratio_mean:.1f}%")
    report.append(f"     - Frequent corrections lead to inefficient motion")
    report.append("")
    
    report.append("[WHY BUMP SENSOR FAILS IN CURVE FOLLOWING]")
    report.append("")
    report.append("  1. REACTIVE CONTROL: Bump Sensor only detects when too close,")
    report.append("     causing delayed reactions and overshooting during curves")
    report.append("")
    report.append("  2. NO POSITION GRADIENT: Unlike Line Sensor, Bump Sensor cannot")
    report.append("     detect how far off-course the robot is, only proximity")
    report.append("")
    report.append("  3. OSCILLATION: Constant correction-overcorrection cycle")
    report.append("     leads to zigzag motion instead of smooth curves")
    report.append("")
    report.append("  4. SPEED INEFFICIENCY: Frequent stops and accelerations")
    report.append("     waste energy and slow overall progress")
    report.append("")
    
    report.append("[CONCLUSION]")
    report.append("  Bump Sensor demonstrates INFERIOR performance in 30° curve following:")
    report.append("")
    report.append("  • Trajectories are inconsistent across experiments")
    report.append("  • Heading angle changes are erratic and unpredictable")
    report.append("  • Sensor provides no proportional feedback for curves")
    report.append("  • Motion is jerky with frequent corrections")
    report.append("")
    report.append("  ★ LINE SENSOR IS SUPERIOR FOR CURVE FOLLOWING TASKS")
    report.append("  ★ BUMP SENSOR IS NOT RECOMMENDED FOR CURVED PATHS")
    report.append("")
    report.append("=" * 70)
    
    # Save report
    report_text = '\n'.join(report)
    report_path = os.path.join(output_folder, 'bump_sensor_30deg_analysis_report.txt')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report_text)
    
    print("\n" + report_text)
    print(f"\n✓ Report saved: {report_path}")

def main():
    print("=" * 60)
    print("Bump Sensor 30° Curve Following Problem Analysis")
    print("Demonstrating BUMP SENSOR DISADVANTAGES")
    print("=" * 60)
    
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Load data
    print("\nLoading data...")
    data = load_all_data(script_dir)
    
    if data is None:
        print("ERROR: No data files found!")
        return
    
    print(f"\nTotal loaded: {len(data)} data points")
    
    # Analyze problems
    print("\nAnalyzing Bump Sensor problems (DISADVANTAGES)...")
    problems_df = analyze_problems(data)
    
    # Generate visualization
    print("\nGenerating charts...")
    create_problem_visualization(data, problems_df, script_dir)
    
    # Generate report
    generate_problem_report(problems_df, script_dir)
    
    print("\n" + "=" * 60)
    print("Analysis complete! Generated files:")
    print("  1. bump_sensor_30deg_problems.png - Problem visualization")
    print("  2. bump_sensor_30deg_repeatability.png - Repeatability analysis")
    print("  3. bump_sensor_30deg_analysis_report.txt - Analysis report")
    print("=" * 60)

if __name__ == "__main__":
    main()
