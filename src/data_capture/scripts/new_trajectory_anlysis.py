import json
import os
import numpy as np
from pathlib import Path
import pandas as pd
from typing import Dict, List, Optional, Tuple
import yaml


class TrajectorySampler:
    def __init__(self, config_path: Optional[str] = None):
        """
        初始化轨迹采样器

        参数:
            config_path: 配置文件路径
        """
        # 默认配置
        self.config = {
            'sampling': {
                'strategy': 'hybrid',  # 'time_based', 'distance_based', 'hybrid'
                'time_interval_us': 500000,  # 时间采样间隔(微秒)
                'min_distance_m': 0.5,  # 最小采样距离(米)
                'future_steps': 12,  # 未来预测步数
                'future_interval_us': 100000,  # 未来轨迹时间间隔(微秒)
                'min_history_frames': 5,  # 最小历史帧数
            },
            'modalities': {
                'rgb': True,
                'depth': True,
                'lidar': False,
                'semantic': False,
                'target_detection': True
            },
            'paths': {
                'rgb_pattern': '/media/gr-agv-a9xy/backup_xxy/camera_data/train/rgb/rgb_{timestamp}.png',
                'depth_pattern': '/media/gr-agv-a9xy/backup_xxy/camera_data/train/depth_vis/depth_{timestamp}.png',
                'output_dir': '/media/gr-agv-x9xy/backup_xxy/camera_data'
            }
        }

        print(f"初始化 TrajectorySampler，配置路径: {config_path}")

        # 加载配置文件
        if config_path and os.path.exists(config_path):
            print(f"加载配置文件: {config_path}")
            try:
                with open(config_path, 'r') as f:
                    user_config = yaml.safe_load(f)
                    self._update_config(user_config)
                print("配置文件加载成功")
            except Exception as e:
                print(f"配置文件加载失败: {e}")
        else:
            if config_path:
                print(f"警告: 配置文件 {config_path} 不存在，使用默认配置")
            else:
                print("使用默认配置")

        print(f"最终配置: {json.dumps(self.config, indent=2, ensure_ascii=False)}")

    def _update_config(self, user_config: Dict):
        """更新配置"""
        print("更新配置...")
        for key, value in user_config.items():
            if isinstance(value, dict) and key in self.config:
                self.config[key].update(value)
                print(f"  更新 {key} 配置")
            else:
                self.config[key] = value
                print(f"  设置 {key} 配置")

    def calculate_distance(self, point1: Dict, point2: Dict) -> float:
        """计算两点之间的欧几里得距离"""
        distance = np.sqrt((point2['x'] - point1['x']) ** 2 +
                           (point2['y'] - point1['y']) ** 2)
        print(
            f"  计算距离: point1({point1['x']:.3f}, {point1['y']:.3f}) -> point2({point2['x']:.3f}, {point2['y']:.3f}) = {distance:.3f}m")
        return distance

    def find_closest_point(self, trajectory: List[Dict], target_time: int) -> Optional[Dict]:
        """找到最接近目标时间的轨迹点"""
        closest_point = None
        min_time_diff = float('inf')

        print(f"  寻找最接近时间 {target_time} 的点")

        for point in trajectory:
            time_diff = abs(point.get('timestamp', 0) - target_time)
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                closest_point = point

        result = closest_point if min_time_diff < self.config['sampling']['future_interval_us'] // 2 else None
        if result:
            print(f"  找到最近点: 时间 {result.get('timestamp', 0)} (差异 {min_time_diff}us)")
        else:
            print(
                f"  未找到合适点，最小时间差异 {min_time_diff}us > 阈值 {self.config['sampling']['future_interval_us'] // 2}us")

        return result

    def can_extract_future(self, trajectory: List[Dict], current_index: int, future_steps: int) -> bool:
        """检查是否能够提取足够的未来轨迹"""
        if current_index + future_steps >= len(trajectory):
            print(
                f"  无法提取未来轨迹: 当前索引 {current_index} + 未来步数 {future_steps} >= 轨迹长度 {len(trajectory)}")
            return False

        current_time = trajectory[current_index].get('timestamp', 0)
        required_time = current_time + future_steps * self.config['sampling']['future_interval_us']

        # 检查最后一个点的时间是否足够
        last_point_time = trajectory[-1].get('timestamp', 0)
        result = last_point_time >= required_time

        if result:
            print(f"  可以提取未来轨迹: 最后时间 {last_point_time} >= 需求时间 {required_time}")
        else:
            print(f"  无法提取未来轨迹: 最后时间 {last_point_time} < 需求时间 {required_time}")

        return result

    def extract_future_trajectory(self, trajectory: List[Dict], current_index: int) -> List[Dict]:
        """提取未来轨迹"""
        print(f"提取未来轨迹，当前索引: {current_index}")
        future_trajectory = []
        current_point = trajectory[current_index]
        current_time = current_point.get('timestamp', 0)
        print(f"  当前点: 时间 {current_time}, 位置 ({current_point['x']:.3f}, {current_point['y']:.3f})")

        for i in range(1, self.config['sampling']['future_steps'] + 1):
            target_time = current_time + i * self.config['sampling']['future_interval_us']
            future_point = self.find_closest_point(trajectory, target_time)

            if future_point:
                # 计算相对坐标
                relative_point = {
                    'x': future_point['x'] - current_point['x'],
                    'y': future_point['y'] - current_point['y'],
                    'timestamp': future_point.get('timestamp', 0),
                    'time_offset': future_point.get('timestamp', 0) - current_time
                }
                future_trajectory.append(relative_point)
                print(
                    f"  未来点 {i}: 相对位置 ({relative_point['x']:.3f}, {relative_point['y']:.3f}), 时间偏移 {relative_point['time_offset']}us")

        print(f"  提取了 {len(future_trajectory)} 个未来点")
        return future_trajectory

    def extract_history_sequence(self, trajectory: List[Dict], current_index: int, seq_length: int) -> List[Dict]:
        """提取历史序列"""
        print(f"提取历史序列，当前索引: {current_index}, 需要长度: {seq_length}")
        history_sequence = []

        # 确保有足够的历史帧
        start_index = max(0, current_index - seq_length + 1)
        print(f"  起始索引: {start_index}, 结束索引: {current_index}")

        for i in range(start_index, current_index + 1):
            current_point = trajectory[i]
            reference_point = trajectory[current_index]

            # 计算相对坐标
            relative_point = {
                'x': current_point['x'] - reference_point['x'],
                'y': current_point['y'] - reference_point['y'],
                'timestamp': current_point.get('timestamp', 0),
                'time_offset': current_point.get('timestamp', 0) - reference_point.get('timestamp', 0)
            }
            history_sequence.append(relative_point)
            print(
                f"  历史点 {i}: 相对位置 ({relative_point['x']:.3f}, {relative_point['y']:.3f}), 时间偏移 {relative_point['time_offset']}us")

        # 如果历史帧不足，用第一帧填充
        while len(history_sequence) < seq_length:
            fill_point = history_sequence[0] if history_sequence else {'x': 0, 'y': 0}
            history_sequence.insert(0, fill_point)
            print(f"  填充历史点: 相对位置 ({fill_point['x']:.3f}, {fill_point['y']:.3f})")

        print(f"  最终历史序列长度: {len(history_sequence)}")
        return history_sequence

    def find_associated_sensor_data(self, trajectory_point: Dict, data_dir: Path) -> Dict:
        """查找关联的传感器数据"""
        timestamp = trajectory_point.get('timestamp', 0)
        sensor_data = {}
        print(f"查找传感器数据，时间戳: {timestamp}")

        # RGB数据
        if self.config['modalities']['rgb']:
            rgb_pattern = self.config['paths']['rgb_pattern'].format(timestamp=timestamp)
            rgb_path = Path(rgb_pattern)
            if rgb_path.exists():
                sensor_data['rgb_path'] = str(rgb_path)
                print(f"  找到RGB数据: {rgb_path}")
            else:
                print(f"  RGB数据不存在: {rgb_path}")

        # 深度数据
        if self.config['modalities']['depth']:
            depth_pattern = self.config['paths']['depth_pattern'].format(timestamp=timestamp)
            depth_path = Path(depth_pattern)
            if depth_path.exists():
                sensor_data['depth_path'] = str(depth_path)
                print(f"  找到深度数据: {depth_path}")
            else:
                print(f"  深度数据不存在: {depth_path}")

        # 目标检测数据
        if self.config['modalities']['target_detection']:
            detection_path = data_dir / 'detections' / f'detection_{timestamp}.json'
            if detection_path.exists():
                sensor_data['detection_path'] = str(detection_path)
                print(f"  找到检测数据: {detection_path}")
            else:
                print(f"  检测数据不存在: {detection_path}")

        print(f"  传感器数据结果: {sensor_data}")
        return sensor_data

    def sample_trajectory(self, trajectory: List[Dict], data_dir: Path) -> List[Dict]:
        """对单条轨迹进行采样"""
        print(f"\n开始采样轨迹，轨迹长度: {len(trajectory)}")
        samples = []

        min_required = self.config['sampling']['min_history_frames'] + self.config['sampling']['future_steps']
        if len(trajectory) < min_required:
            print(f"轨迹长度不足: {len(trajectory)} < {min_required} (min_history_frames + future_steps)")
            return samples

        # 检查轨迹数据格式
        for i, point in enumerate(trajectory[:3]):  # 只检查前3个点
            if 'x' not in point or 'y' not in point:
                print(f"轨迹点 {i} 格式错误: 缺少x或y坐标")
                return samples

        print(f"轨迹时间范围: {trajectory[0].get('timestamp', 0)} - {trajectory[-1].get('timestamp', 0)}")
        print(f"采样策略: {self.config['sampling']['strategy']}")
        print(f"时间间隔: {self.config['sampling']['time_interval_us']}us")
        print(f"最小距离: {self.config['sampling']['min_distance_m']}m")

        last_sample_index = 0
        sample_count = 0

        for i in range(1, len(trajectory)):
            current_point = trajectory[i]
            last_sample_point = trajectory[last_sample_index]

            # 计算采样条件
            time_diff = current_point.get('timestamp', 0) - last_sample_point.get('timestamp', 0)
            distance = self.calculate_distance(last_sample_point, current_point)

            time_condition = time_diff >= self.config['sampling']['time_interval_us']
            distance_condition = distance >= self.config['sampling']['min_distance_m']

            # 根据采样策略决定是否采样
            should_sample = False
            strategy = self.config['sampling']['strategy']

            if strategy == 'time_based':
                should_sample = time_condition
            elif strategy == 'distance_based':
                should_sample = distance_condition
            elif strategy == 'hybrid':
                should_sample = time_condition or distance_condition

            print(f"\n检查点 {i}: 时间差 {time_diff}us, 距离 {distance:.3f}m")
            print(f"  时间条件: {time_condition}, 距离条件: {distance_condition}, 应该采样: {should_sample}")

            # 检查是否能提取未来轨迹
            if should_sample:
                can_extract = self.can_extract_future(trajectory, i, self.config['sampling']['future_steps'])
                print(f"  能否提取未来轨迹: {can_extract}")

                if can_extract:
                    # 提取未来轨迹
                    future_trajectory = self.extract_future_trajectory(trajectory, i)

                    # 提取历史序列
                    history_sequence = self.extract_history_sequence(trajectory, i,
                                                                     self.config['sampling']['min_history_frames'])

                    # 查找关联的传感器数据
                    sensor_data = self.find_associated_sensor_data(trajectory[i], data_dir)

                    # 创建样本
                    sample = {
                        'sample_id': f"{sample_count:08d}",
                        'trajectory_index': i,
                        'timestamp': trajectory[i].get('timestamp', 0),
                        'absolute_position': {
                            'x': trajectory[i]['x'],
                            'y': trajectory[i]['y']
                        },
                        'history_sequence': history_sequence,
                        'future_trajectory': future_trajectory,
                        'sensor_data': sensor_data,
                        'metadata': {
                            'time_since_last_sample': time_diff,
                            'distance_since_last_sample': distance,
                            'velocity': self.calculate_velocity(trajectory, i)
                        }
                    }

                    samples.append(sample)
                    last_sample_index = i
                    sample_count += 1
                    print(f"  创建样本 #{sample_count} 成功!")
                else:
                    print("  无法提取未来轨迹，跳过采样")
            else:
                print("  不满足采样条件，跳过")

        print(f"轨迹采样完成，生成 {len(samples)} 个样本")
        return samples

    def calculate_velocity(self, trajectory: List[Dict], current_index: int) -> float:
        """计算当前速度"""
        if current_index < 1:
            return 0.0

        prev_point = trajectory[current_index - 1]
        curr_point = trajectory[current_index]

        time_diff = (curr_point.get('timestamp', 0) - prev_point.get('timestamp', 0)) / 1000000 # 转换为秒
        if time_diff <= 0:
            return 0.0

        distance = self.calculate_distance(prev_point, curr_point)
        velocity = distance / time_diff
        print(f"  计算速度: 距离 {distance:.3f}m / 时间 {time_diff:.3f}s = {velocity:.3f}m/s")
        return velocity

    def process_dataset(self, trajectory_dir: Path, output_dir: Optional[Path] = None):
        """处理整个数据集"""
        print(f"\n开始处理数据集")
        print(f"轨迹目录: {trajectory_dir}")

        # 初始化统计信息
        statistics = {
            'total_trajectories': 0,
            'total_samples': 0,
            'samples_per_trajectory': [],
            'modality_availability': {'rgb': 0, 'depth': 0, 'detection': 0}
        }

        if output_dir is None:
            output_dir = Path(self.config['paths']['output_dir'])
        print(f"输出目录: {output_dir}")

        # 检查目录是否存在
        if not trajectory_dir.exists():
            print(f"错误: 轨迹目录 {trajectory_dir} 不存在!")
            # 返回空的样本列表和统计信息
            return [], statistics

        # 确保输出目录存在
        try:
            output_dir.mkdir(exist_ok=True, parents=True)
            print(f"确保输出目录存在: {output_dir}")
        except Exception as e:
            print(f"创建输出目录失败: {e}")
            return [], statistics

        trajectory_files = list(trajectory_dir.glob("*.json"))
        print(f"找到 {len(trajectory_files)} 个JSON文件")

        if not trajectory_files:
            print("警告: 未找到任何JSON轨迹文件")
            return [], statistics

        all_samples = []
        statistics['total_trajectories'] = len(trajectory_files)

        for traj_file in trajectory_files:
            print(f"\n处理文件: {traj_file.name}")
            try:
                with open(traj_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                print(f"  JSON文件加载成功")

                trajectory = data.get('recent_trajectory', [])
                print(f"  轨迹长度: {len(trajectory)} 个点")

                if not trajectory:
                    print(f"  警告: 文件 {traj_file.name} 中没有轨迹数据")
                    continue

                # 检查轨迹数据格式
                first_point = trajectory[0]
                if 'x' not in first_point or 'y' not in first_point:
                    print(f"  错误: 轨迹数据格式不正确，缺少x或y坐标")
                    continue

                # 对当前轨迹采样
                samples = self.sample_trajectory(trajectory, traj_file.parent)

                # 更新统计信息
                statistics['total_samples'] += len(samples)
                statistics['samples_per_trajectory'].append(len(samples))

                # 更新模态可用性统计
                for sample in samples:
                    if 'rgb_path' in sample['sensor_data']:
                        statistics['modality_availability']['rgb'] += 1
                    if 'depth_path' in sample['sensor_data']:
                        statistics['modality_availability']['depth'] += 1
                    if 'detection_path' in sample['sensor_data']:
                        statistics['modality_availability']['detection'] += 1

                all_samples.extend(samples)

                print(f"  文件处理完成: {len(samples)} 个样本")

            except json.JSONDecodeError as e:
                print(f"  JSON解析错误: {e}")
            except Exception as e:
                print(f"  处理文件时出错: {e}")
                import traceback
                traceback.print_exc()

        print(f"\n数据集处理完成")
        print(f"总轨迹数: {statistics['total_trajectories']}")
        print(f"总样本数: {statistics['total_samples']}")

        # 保存结果 - 添加异常处理
        try:
            if all_samples:
                # 确保输出目录存在
                output_dir.mkdir(exist_ok=True, parents=True)
                output_file = output_dir / 'sampled_trajectories.json'

                print(f"保存结果到: {output_file}")
                print(f"准备保存 {len(all_samples)} 个样本")

                result_data = {
                    'samples': all_samples,
                    'statistics': statistics,
                    'config': self.config
                }

                with open(output_file, 'w', encoding='utf-8') as f:
                    json.dump(result_data, f, indent=2, ensure_ascii=False)

                # 验证保存结果
                if output_file.exists():
                    file_size = output_file.stat().st_size
                    print(f"文件保存成功! 文件大小: {file_size} 字节")

                    # 可选：验证读取
                    try:
                        with open(output_file, 'r', encoding='utf-8') as f:
                            verification_data = json.load(f)
                        saved_samples = len(verification_data.get('samples', []))
                        print(f"验证读取: {saved_samples} 个样本")
                    except Exception as e:
                        print(f"验证读取失败: {e}")
                else:
                    print("错误: 文件保存失败!")

                # 保存统计摘要
                self.save_statistics_summary(statistics, output_dir)
                print("统计摘要已保存")
            else:
                print("警告: 未生成任何样本，跳过保存")

        except Exception as e:
            print(f"保存结果时出错: {e}")
            import traceback
            traceback.print_exc()

        # 确保返回结果
        return all_samples, statistics


# 使用示例
# 使用示例
if __name__ == "__main__":
    config_path = Path('/home/gr-agv-x9xy/isaac_sim_ws/src/data_capture/config/trajectory_ansysis.yaml')

    print("=" * 60)
    print("Trajectory Sampler 启动")
    print("=" * 60)

    try:
        # 初始化采样器
        sampler = TrajectorySampler(str(config_path) if config_path.exists() else None)

        # 处理数据集
        trajectory_dir = Path("/media/gr-agv-x9xy/backup_xxy/camera_data/train/trajectory")
        output_dir = Path("/media/gr-agv-x9xy/backup_xxy/camera_data/processed_samples")

        print(f"\n开始处理主数据集")
        result = sampler.process_dataset(trajectory_dir, output_dir)

        # 检查返回值
        if result is None:
            print("错误: process_dataset 返回了 None")
            samples, stats = [], {}
        else:
            samples, stats = result

        print(f"\n处理完成!")
        print(f"总轨迹数: {stats.get('total_trajectories', 0)}")
        print(f"总样本数: {stats.get('total_samples', 0)}")

        if stats.get('samples_per_trajectory'):
            avg_samples = np.mean(stats['samples_per_trajectory'])
            print(f"平均每条轨迹样本数: {avg_samples:.2f}")
        else:
            print("平均每条轨迹样本数: 0")

    except Exception as e:
        print(f"程序执行出错: {e}")
        import traceback

        traceback.print_exc()

    print("=" * 60)