import json
import os
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import pandas as pd


def calculate_trajectory_length(trajectory_data):
    """
    计算轨迹长度（欧几里得距离累加）
    """
    total_length = 0.0
    if not trajectory_data or len(trajectory_data) < 2:
        return 0.0

    for i in range(1, len(trajectory_data)):
        point1 = trajectory_data[i - 1]
        point2 = trajectory_data[i]

        # 计算两点间距离
        distance = np.sqrt((point2['x'] - point1['x']) ** 2 +
                           (point2['y'] - point1['y']) ** 2)
        total_length += distance

    return total_length


def analyze_trajectory_files(trajectory_dir):
    """
    分析轨迹文件夹中的所有JSON文件
    """
    trajectory_dir = Path(trajectory_dir)
    json_files = list(trajectory_dir.glob("*.json"))

    lengths = []
    file_info = []
    error_files = []

    for json_file in json_files:
        try:
            with open(json_file, 'r', encoding='utf-8') as f:
                data = json.load(f)

            # 获取实际轨迹数据，处理None值的情况
            trajectory_data = data.get('recent_trajectory', [])
            if trajectory_data is None:
                trajectory_data = []

            length = calculate_trajectory_length(trajectory_data)

            # 获取规划轨迹数据，处理None值的情况
            planned_trajectory = data.get('planned_trajectory', [])
            if planned_trajectory is None:
                planned_trajectory = []

            lengths.append(length)
            file_info.append({
                'filename': json_file.name,
                'length': length,
                'num_points': len(trajectory_data),
                'has_plan': len(planned_trajectory) > 0,
                'plan_length': len(planned_trajectory),
                'episode_id': data.get('episode_id')
            })

        except Exception as e:
            print(f"处理文件 {json_file.name} 时出错: {e}")
            error_files.append(json_file.name)

    return lengths, file_info, error_files


def create_visualizations(lengths, file_info):
    """
    创建多种可视化图表（使用英文标签）
    """
    # 设置字体 - 使用英文不需要中文字体
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Helvetica']
    plt.rcParams['axes.unicode_minus'] = False

    # 转换为DataFrame便于分析
    df = pd.DataFrame(file_info) if file_info else pd.DataFrame()

    # 创建图表
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle('Trajectory Analysis Report', fontsize=16, fontweight='bold')

    # 1. 轨迹长度分布直方图
    if lengths:
        axes[0, 0].hist(lengths, bins=30, alpha=0.7, color='skyblue', edgecolor='black')
        axes[0, 0].set_xlabel('Trajectory Length (m)')
        axes[0, 0].set_ylabel('Frequency')
        axes[0, 0].set_title('Trajectory Length Distribution')
        axes[0, 0].axvline(np.mean(lengths), color='red', linestyle='--', label=f'Mean: {np.mean(lengths):.2f}m')
        axes[0, 0].axvline(np.median(lengths), color='green', linestyle='--',
                           label=f'Median: {np.median(lengths):.2f}m')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
    else:
        axes[0, 0].text(0.5, 0.5, 'No trajectory data', ha='center', va='center', transform=axes[0, 0].transAxes)
        axes[0, 0].set_title('Trajectory Length Distribution')

    # 2. 轨迹长度箱线图
    if lengths:
        axes[0, 1].boxplot(lengths, vert=True)
        axes[0, 1].set_ylabel('Trajectory Length (m)')
        axes[0, 1].set_title('Trajectory Length Boxplot')
        axes[0, 1].grid(True, alpha=0.3)
    else:
        axes[0, 1].text(0.5, 0.5, 'No trajectory data', ha='center', va='center', transform=axes[0, 1].transAxes)
        axes[0, 1].set_title('Trajectory Length Boxplot')

    # 3. 轨迹点数与长度的关系
    if not df.empty and 'num_points' in df.columns and 'length' in df.columns:
        axes[1, 0].scatter(df['num_points'], df['length'], alpha=0.6, color='blue')
        axes[1, 0].set_xlabel('Number of Points')
        axes[1, 0].set_ylabel('Trajectory Length (m)')
        axes[1, 0].set_title('Points Count vs Length')
        axes[1, 0].grid(True, alpha=0.3)

        # 添加趋势线（简单线性拟合）
        if len(df) > 1:
            x = df['num_points']
            y = df['length']
            coef = np.polyfit(x, y, 1)
            poly1d_fn = np.poly1d(coef)
            axes[1, 0].plot(x, poly1d_fn(x), 'r--', alpha=0.8)
    else:
        axes[1, 0].text(0.5, 0.5, 'No data or incomplete data', ha='center', va='center',
                        transform=axes[1, 0].transAxes)
        axes[1, 0].set_title('Points Count vs Length')

    # 4. 轨迹长度累积分布图
    if lengths:
        sorted_lengths = np.sort(lengths)
        cum_prob = np.arange(1, len(sorted_lengths) + 1) / len(sorted_lengths)
        axes[1, 1].plot(sorted_lengths, cum_prob, 'b-', linewidth=2)
        axes[1, 1].set_xlabel('Trajectory Length (m)')
        axes[1, 1].set_ylabel('Cumulative Probability')
        axes[1, 1].set_title('Cumulative Distribution of Length')
        axes[1, 1].grid(True, alpha=0.3)
    else:
        axes[1, 1].text(0.5, 0.5, 'No trajectory data', ha='center', va='center', transform=axes[1, 1].transAxes)
        axes[1, 1].set_title('Cumulative Distribution of Length')

    plt.tight_layout()
    plt.savefig('trajectory_analysis_report.png', dpi=300, bbox_inches='tight')
    plt.show()

    # 如果有规划数据，创建额外的饼图
    if not df.empty and 'has_plan' in df.columns:
        create_plan_ratio_chart(df)

    # 创建统计摘要
    print_statistical_summary(lengths, file_info)


def create_plan_ratio_chart(df):
    """
    创建规划轨迹比例饼图（使用英文标签）
    """
    plan_counts = df['has_plan'].value_counts()

    # 确保有足够的数据
    if len(plan_counts) > 0:
        fig, ax = plt.subplots(figsize=(8, 6))

        # 根据实际数据情况设置标签
        labels = []
        sizes = []
        colors = []

        if True in plan_counts.index:
            labels.append('With Planned Trajectory')
            sizes.append(plan_counts[True])
            colors.append('lightgreen')

        if False in plan_counts.index:
            labels.append('Without Planned Trajectory')
            sizes.append(plan_counts[False])
            colors.append('lightcoral')

        if sizes:
            ax.pie(sizes, labels=labels, autopct='%1.1f%%', colors=colors, startangle=90)
            ax.set_title('Files with Planned Trajectory')
            plt.savefig('trajectory_plan_ratio.png', dpi=300, bbox_inches='tight')
            plt.close(fig)


def print_statistical_summary(lengths, file_info):
    """
    打印统计摘要
    """
    print("\n" + "=" * 50)
    print("Trajectory Data Statistical Summary")
    print("=" * 50)

    if lengths:
        print(f"Total trajectories: {len(lengths)}")
        print(f"Average length: {np.mean(lengths):.3f} meters")
        print(f"Max length: {np.max(lengths):.3f} meters")
        print(f"Min length: {np.min(lengths):.3f} meters")
        print(f"Median length: {np.median(lengths):.3f} meters")
        print(f"Standard deviation: {np.std(lengths):.3f} meters")
        print(f"25th percentile: {np.percentile(lengths, 25):.3f} meters")
        print(f"75th percentile: {np.percentile(lengths, 75):.3f} meters")
    else:
        print("No valid trajectory data")

    if file_info:
        df = pd.DataFrame(file_info)
        if 'has_plan' in df.columns:
            has_plan_count = df['has_plan'].sum()
            print(f"Files with planned trajectory: {has_plan_count}")
            print(f"Percentage with planned trajectory: {has_plan_count / len(df) * 100:.1f}%")

        if 'num_points' in df.columns:
            print(f"Average points per trajectory: {df['num_points'].mean():.1f}")
            print(f"Max points: {df['num_points'].max()}")
            print(f"Min points: {df['num_points'].min()}")


# 使用示例
if __name__ == "__main__":
    trajectory_dir = "/media/gr-agv-x9xy/backup_xxy/camera_data/train/trajectory"

    # 检查目录是否存在
    if not os.path.exists(trajectory_dir):
        print(f"Error: Directory {trajectory_dir} does not exist")
        exit(1)

    # 分析数据
    lengths, file_info, error_files = analyze_trajectory_files(trajectory_dir)

    if error_files:
        print(f"\nNumber of files with errors: {len(error_files)}")

    if not lengths:
        print("No valid trajectory data found")
        exit(1)

    # 创建可视化图表
    create_visualizations(lengths, file_info)

    # 保存详细统计信息到CSV文件
    if file_info:
        df = pd.DataFrame(file_info)
        df.to_csv('trajectory_statistics.csv', index=False, encoding='utf-8-sig')
        print("\nDetailed statistics saved to trajectory_statistics.csv")

    print("\nAnalysis completed!")