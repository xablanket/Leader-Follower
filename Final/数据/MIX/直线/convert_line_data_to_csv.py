import csv
import re
import os

def extract_csv_from_line_data(input_file, output_file):
    """
    从串口输出文件中提取 CSV 数据并保存为标准 CSV 文件
    支持 Bump+Line 混合版本和纯 Line 版本
    """
    with open(input_file, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    csv_lines = []
    found_header = False
    
    for line in lines:
        cleaned_line = re.sub(r'^\s*\d{2}:\d{2}:\d{2}\.\d{3}\s*->\s*', '', line)
        cleaned_line = cleaned_line.strip()
        
        if not cleaned_line:
            continue
        
        if '==========' in cleaned_line and found_header:
            break
        
        if any(keyword in cleaned_line for keyword in [
            'Data Description', 'Total samples', 'Record interval',
            'Experiment finished', 'Position (mm)', 'Heading angle',
            'Wheel speed', 'sensor', 'command'
        ]):
            continue
        
        if cleaned_line.startswith('Sample,X_mm,Y_mm'):
            csv_lines.append(cleaned_line)
            found_header = True
            continue
        
        if found_header and ',' in cleaned_line and cleaned_line[0].isdigit():
            csv_lines.append(cleaned_line)
    
    with open(output_file, 'w', encoding='utf-8', newline='') as f:
        for l in csv_lines:
            f.write(l + '\n')
    
    print(f"✓ 成功将数据保存到 {output_file}")
    print(f"  - 共 {len(csv_lines)} 行数据")
    print(f"  - 样本数: {len(csv_lines) - 1}")


if __name__ == "__main__":
    import sys

    # Windows UTF-8 输出
    if sys.platform == 'win32':
        import io
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

    print("\n" + "=" * 80)
    print("CSV 数据提取工具 - 批量处理模式")
    print("=" * 80)

    # ★★★ 关键改动：切换到脚本所在目录 ★★★
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    print(f"脚本所在目录: {script_dir}")
    print("（已自动切换到此目录执行）\n")

    # 自动检测 1.txt ~ 10.txt
    found_files = []
    for i in range(1, 11):
        txt_file = f"{i}.txt"
        if os.path.exists(txt_file):
            found_files.append(txt_file)

    if not found_files:
        print("未找到编号文件 (1.txt ~ 10.txt)")
        print("\n尝试处理单个文件 'Line.txt'...\n")

        input_file = "Line.txt"
        output_file = "line_data.csv"

        try:
            extract_csv_from_line_data(input_file, output_file)
            print("\n预览前 5 行数据：")
            print("-" * 80)
            with open(output_file, 'r', encoding='utf-8') as f:
                for i, line in enumerate(f):
                    if i < 5:
                        print(line.strip())
                    else:
                        break
        except FileNotFoundError:
            print(f"错误：找不到 {input_file}")
            print("\n使用方法：")
            print("  1) 把脚本和 1.txt ~ 10.txt 放在同一文件夹")
            print("  2) 或放置 Line.txt 并运行脚本")
        except Exception as e:
            print(f"错误：{e}")

    else:
        print(f"找到 {len(found_files)} 个文件：")
        for f in found_files:
            print("  -", f)
        print("")

        success_count = 0
        failed_files = []

        for txt_file in found_files:
            output_file = txt_file.replace('.txt', '.csv')
            try:
                print(f"处理: {txt_file} -> {output_file}")
                extract_csv_from_line_data(txt_file, output_file)
                success_count += 1
            except Exception as e:
                print(f"✗ 失败: {txt_file} | {e}")
                failed_files.append(txt_file)

        print("\n" + "=" * 80)
        print(f"批量处理完成: 成功 {success_count}/{len(found_files)}")
        if failed_files:
            print("失败文件:")
            for f in failed_files:
                print("  -", f)
        print("=" * 80)
