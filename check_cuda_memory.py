#!/usr/bin/env python3
"""
CUDA内存检查和管理工具
用于查看GPU内存使用情况和清理内存
"""
import torch
import subprocess
import sys


def print_memory_info():
    """打印详细的CUDA内存信息"""
    if not torch.cuda.is_available():
        print("❌ CUDA不可用")
        return
    
    print("=" * 60)
    print("CUDA 内存信息")
    print("=" * 60)
    
    # PyTorch内存信息
    device = torch.device('cuda:0')
    allocated = torch.cuda.memory_allocated(device) / 1024**3  # GB
    reserved = torch.cuda.memory_reserved(device) / 1024**3  # GB
    max_allocated = torch.cuda.max_memory_allocated(device) / 1024**3  # GB
    max_reserved = torch.cuda.max_memory_reserved(device) / 1024**3  # GB
    
    print(f"\n📊 PyTorch内存统计 (GPU 0):")
    print(f"  已分配内存: {allocated:.2f} GB")
    print(f"  预留内存: {reserved:.2f} GB")
    print(f"  最大已分配: {max_allocated:.2f} GB")
    print(f"  最大预留: {max_reserved:.2f} GB")
    
    # nvidia-smi信息
    try:
        result = subprocess.run(['nvidia-smi', '--query-gpu=index,name,memory.total,memory.used,memory.free', 
                                '--format=csv,noheader,nounits'], 
                               capture_output=True, text=True, check=True)
        print(f"\n📈 nvidia-smi 信息:")
        lines = result.stdout.strip().split('\n')
        for line in lines:
            parts = line.split(', ')
            if len(parts) >= 5:
                idx, name, total, used, free = parts[0], parts[1], parts[2], parts[3], parts[4]
                total_gb = float(total) / 1024
                used_gb = float(used) / 1024
                free_gb = float(free) / 1024
                print(f"  GPU {idx} ({name}):")
                print(f"    总内存: {total_gb:.2f} GB")
                print(f"    已使用: {used_gb:.2f} GB ({used_gb/total_gb*100:.1f}%)")
                print(f"    空闲: {free_gb:.2f} GB ({free_gb/total_gb*100:.1f}%)")
                
                # 检查是否有其他进程
                result_proc = subprocess.run(['nvidia-smi', '--query-compute-apps=pid,process_name,used_memory', 
                                             '--format=csv,noheader,nounits'], 
                                            capture_output=True, text=True, check=True)
                if result_proc.stdout.strip():
                    print(f"\n  正在运行的进程:")
                    for proc_line in result_proc.stdout.strip().split('\n'):
                        proc_parts = proc_line.split(', ')
                        if len(proc_parts) >= 3:
                            pid, name, mem = proc_parts[0], proc_parts[1], proc_parts[2]
                            mem_gb = float(mem) / 1024
                            print(f"    PID {pid}: {name} ({mem_gb:.2f} GB)")
    except subprocess.CalledProcessError as e:
        print(f"⚠️  无法获取nvidia-smi信息: {e}")
    except FileNotFoundError:
        print("⚠️  nvidia-smi命令不可用")
    
    print("=" * 60)


def clear_cache():
    """清理CUDA缓存"""
    if not torch.cuda.is_available():
        print("❌ CUDA不可用，无法清理缓存")
        return
    
    print("\n🧹 清理CUDA缓存...")
    
    # 清空缓存
    torch.cuda.empty_cache()
    
    # 重置峰值统计
    torch.cuda.reset_peak_memory_stats()
    
    # 同步
    torch.cuda.synchronize()
    
    allocated = torch.cuda.memory_allocated() / 1024**3
    reserved = torch.cuda.memory_reserved() / 1024**3
    
    print(f"✅ 缓存已清理")
    print(f"   当前已分配: {allocated:.2f} GB")
    print(f"   当前预留: {reserved:.2f} GB")


def kill_processes_by_name(process_names):
    """根据进程名杀死进程"""
    import psutil
    killed = []
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            cmdline = ' '.join(proc.info['cmdline'] or [])
            for name in process_names:
                if name in cmdline:
                    proc.kill()
                    killed.append((proc.info['pid'], proc.info['name']))
                    print(f"✅ 已终止进程: PID {proc.info['pid']} ({proc.info['name']})")
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return killed


if __name__ == "__main__":
    if len(sys.argv) > 1:
        command = sys.argv[1]
        
        if command == "info" or command == "i":
            print_memory_info()
        elif command == "clear" or command == "c":
            clear_cache()
            print_memory_info()
        elif command == "kill":
            print("🔍 查找Python训练进程...")
            killed = kill_processes_by_name(['train.py', 'navdp_train.py', 'python'])
            if not killed:
                print("ℹ️  没有找到相关进程")
        elif command == "all" or command == "a":
            print("🔍 查找并终止相关进程...")
            kill_processes_by_name(['train.py', 'navdp_train.py'])
            print("\n🧹 清理缓存...")
            clear_cache()
            print("\n📊 当前内存状态:")
            print_memory_info()
        else:
            print(f"❌ 未知命令: {command}")
            print("\n用法:")
            print("  python check_cuda_memory.py info   - 查看内存信息")
            print("  python check_cuda_memory.py clear  - 清理缓存")
            print("  python check_cuda_memory.py kill   - 终止相关进程")
            print("  python check_cuda_memory.py all    - 执行所有操作")
    else:
        print_memory_info()

