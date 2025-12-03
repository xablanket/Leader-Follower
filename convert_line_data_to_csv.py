import csv
import re

def extract_csv_from_line_data(input_file, output_file):
    """
    从串口输出文件中提取 CSV 数据并保存为标准 CSV 文件
    支持 Bump+Line 混合版本和纯 Line 版本
    
    Args:
        input_file: 输入的文本文件路径
        output_file: 输出的 CSV 文件路径
    """
    with open(input_file, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    # 提取 CSV 数据行
    csv_lines = []
    found_header = False
    
    for line in lines:
        # 移除时间戳前缀
        cleaned_line = re.sub(r'^\s*\d{2}:\d{2}:\d{2}\.\d{3}\s*->\s*', '', line)
        cleaned_line = cleaned_line.strip()
        
        # 跳过空行
        if not cleaned_line:
            continue
        
        # 遇到分隔线停止（数据结束标志）
        if '==========' in cleaned_line and found_header:
            break
        
        # 跳过说明文字
        if any(keyword in cleaned_line for keyword in ['Data Description', 'Total samples', 'Record interval', 'Experiment finished', 'Position (mm)', 'Heading angle', 'Wheel speed', 'sensor', 'command']):
            continue
        
        # 找到表头行（包含 Sample,X_mm,Y_mm）
        if cleaned_line.startswith('Sample,X_mm,Y_mm'):
            csv_lines.append(cleaned_line)
            found_header = True
            continue
        
        # 提取数据行（必须以数字开头，包含逗号，且已找到表头）
        if found_header and ',' in cleaned_line and cleaned_line[0].isdigit():
            csv_lines.append(cleaned_line)
    
    # 保存为 CSV 文件
    with open(output_file, 'w', encoding='utf-8', newline='') as f:
        for line in csv_lines:
            f.write(line + '\n')
    
    print(f"✓ 成功将数据保存到 {output_file}")
    print(f"  - 共 {len(csv_lines)} 行数据（包含表头）")
    print(f"  - 共 {len(csv_lines) - 1} 条样本记录")

if __name__ == "__main__":
    import sys
    import os
    
    # 设置输出编码（Windows 兼容性）
    if sys.platform == 'win32':
        import io
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
    
    print("\n" + "=" * 80)
    print("CSV 数据提取工具 - 批量处理模式")
    print("=" * 80)
    
    # 自动检测当前目录下的 1.txt ~ 10.txt 文件
    current_dir = os.getcwd()
    print(f"当前目录: {current_dir}\n")
    
    # 检测 1.txt 到 10.txt
    found_files = []
    for i in range(1, 11):
        txt_file = f"{i}.txt"
        if os.path.exists(txt_file):
            found_files.append(txt_file)
    
    if not found_files:
        print("未找到编号文件 (1.txt ~ 10.txt)")
        print("\n尝试处理单个文件 'Line.txt'...")
        
        # 回退到单文件模式
        input_file = "Line.txt"
        output_file = "line_data.csv"
        
        try:
            extract_csv_from_line_data(input_file, output_file)
            
            # 预览数据
            print("\n预览前 5 行数据：")
            print("-" * 80)
            with open(output_file, 'r', encoding='utf-8') as f:
                for i, line in enumerate(f):
                    if i < 5:
                        print(line.strip())
                    else:
                        break
            
        except FileNotFoundError:
            print(f"错误：找不到文件 '{input_file}'")
            print("\n使用方法：")
            print("  将脚本放在包含 1.txt ~ 10.txt 的文件夹中运行")
            print("  或在当前目录放置 Line.txt 文件")
        except Exception as e:
            print(f"错误：{e}")
    
    else:
        # 批量处理模式
        print(f"找到 {len(found_files)} 个文件:")
        for f in found_files:
            print(f"  - {f}")
        print("")
        
        success_count = 0
        failed_files = []
        
        for txt_file in found_files:
            input_file = txt_file
            output_file = txt_file.replace('.txt', '.csv')
            
            try:
                print(f"处理: {txt_file} -> {output_file}")
                extract_csv_from_line_data(input_file, output_file)
                success_count += 1
            except Exception as e:
                print(f"  ✗ 失败: {e}")
                failed_files.append(txt_file)
        
        print("\n" + "=" * 80)
        print(f"批量处理完成！")
        print(f"  成功: {success_count}/{len(found_files)} 个文件")
        
        if failed_files:
            print(f"  失败: {len(failed_files)} 个文件")
            for f in failed_files:
                print(f"    - {f}")
        print("=" * 80)

