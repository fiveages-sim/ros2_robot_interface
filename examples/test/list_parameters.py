"""
ROS2 硬件系统参数查询测试脚本

测试 ROS2RobotInterface 的 list_nodes() 和 list_node_parameters() 方法，
用于查找所有包含 "system" 的节点，并查询显示这些节点的参数。
此测试可以在接口连接或未连接状态下运行。
"""

import time
import sys

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def format_parameter_value(value, max_length=80):
    """格式化参数值，用于显示"""
    if value is None:
        return "None"
    
    if isinstance(value, (list, tuple)):
        if len(value) == 0:
            return "[]"
        elif len(value) <= 3:
            return str(value)
        else:
            return f"[{value[0]}, {value[1]}, ..., {value[-1]}] (共 {len(value)} 项)"
    
    value_str = str(value)
    if len(value_str) > max_length:
        return value_str[:max_length] + "..."
    return value_str


def get_user_choice():
    """获取用户选择：查看hardware还是controller的参数"""
    print("\n" + "=" * 70)
    print(" " * 15 + "ROS2 参数查询工具")
    print("=" * 70 + "\n")
    print("  请选择要查询的参数类型：")
    print("  1. Hardware 参数")
    print("  2. Controller 参数")
    print("  3. 退出")
    print()
    
    while True:
        try:
            choice = input("  请输入选项 (1/2/3): ").strip()
            if choice == '1':
                return 'hardware'
            elif choice == '2':
                return 'controller'
            elif choice == '3':
                return 'exit'
            else:
                print("  ⚠ 无效选项，请输入 1、2 或 3")
        except (EOFError, KeyboardInterrupt):
            print("\n  用户取消操作")
            return 'exit'


def main():
    """测试 ROS2 硬件系统参数查询功能"""
    
    # 获取用户选择
    param_type = get_user_choice()
    if param_type == 'exit':
        print("\n  已退出\n")
        return 0
    
    # 根据选择设置显示标题和筛选关键词
    if param_type == 'hardware':
        title = "ROS2 Hardware Parameters Test"
        keyword = 'system'  # hardware system 使用 'system' 作为查询关键词
    else:  # controller
        title = "ROS2 Controller Parameters Test"
        keyword = 'controller'
    
    print("\n" + "=" * 70)
    print(" " * (35 - len(title) // 2) + title)
    print("=" * 70 + "\n")
    
    # ========================================================================
    # 第一部分：创建接口并查询节点
    # ========================================================================
    print("-" * 70)
    print(f"[1] 查询包含 '{keyword}' 的节点")
    print("-" * 70)
    
    # 创建配置对象
    config = ROS2RobotInterfaceConfig()
    
    # 创建接口实例（不连接）
    print("  → 创建 ROS2RobotInterface 实例...")
    interface = ROS2RobotInterface(config)
    
    # 查询节点列表
    print("  → 查询节点列表...")
    try:
        nodes = interface.list_nodes()
        print(f"  ✓ 成功查询到 {len(nodes)} 个节点\n")
    except Exception as e:
        print(f"  ✗ 查询节点列表失败: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # 筛选包含关键词的节点（不区分大小写）
    filtered_nodes = [
        node for node in nodes 
        if keyword in node['name'].lower() or keyword in node['full_name'].lower()
    ]
    
    system_nodes = filtered_nodes
    
    if not system_nodes:
        print(f"  ⚠ 未发现包含 '{keyword}' 的节点")
        print("\n" + "=" * 70)
        print("\n  测试完成（未找到相关节点）\n")
        return 0
    
    print(f"  ✓ 找到 {len(system_nodes)} 个包含 '{keyword}' 的节点:\n")
    for i, node in enumerate(system_nodes, 1):
        print(f"  {i}. {node['full_name']}")
        print(f"     名称: {node['name']}, 命名空间: {node['namespace']}")
    print()
    
    # ========================================================================
    # 第二部分：查询每个节点的参数
    # ========================================================================
    print("-" * 70)
    print("[2] 查询节点参数")
    print("-" * 70)
    
    all_node_params = {}
    
    for idx, node in enumerate(system_nodes, 1):
        print(f"\n  [{idx}/{len(system_nodes)}] 查询节点: {node['full_name']}")
        print("  " + "-" * 66)
        
        try:
            # 使用完整节点名称查询参数
            full_node_name = node['full_name']
            params = interface.list_node_parameters(full_node_name)
            
            if params:
                print(f"  ✓ 找到 {len(params)} 个参数:\n")
                all_node_params[node['full_name']] = params
                
                # 显示参数信息
                for param in params:
                    print(f"    参数名: {param['name']}")
                    print(f"      类型: {param['type']}")
                    print(f"      值: {format_parameter_value(param['value'])}")
                    if param.get('description'):
                        desc = param['description']
                        if len(desc) > 60:
                            desc = desc[:60] + "..."
                        print(f"      描述: {desc}")
                    if param.get('read_only'):
                        print(f"      只读: 是")
                    if param.get('dynamic_typing'):
                        print(f"      动态类型: 是")
                    print()
            else:
                print(f"  ⚠ 该节点没有可配置参数")
                all_node_params[node['full_name']] = []
                
        except Exception as e:
            print(f"  ✗ 查询参数失败: {e}")
            import traceback
            traceback.print_exc()
            all_node_params[node['full_name']] = None
    
    print()
    
    # ========================================================================
    # 第三部分：统计信息
    # ========================================================================
    print("-" * 70)
    print("[3] 统计信息")
    print("-" * 70)
    
    total_params = 0
    nodes_with_params = 0
    nodes_without_params = 0
    nodes_with_errors = 0
    
    for node_name, params in all_node_params.items():
        if params is None:
            nodes_with_errors += 1
        elif len(params) > 0:
            nodes_with_params += 1
            total_params += len(params)
        else:
            nodes_without_params += 1
    
    print(f"  总节点数: {len(system_nodes)}")
    print(f"  有参数的节点: {nodes_with_params}")
    print(f"  无参数的节点: {nodes_without_params}")
    if nodes_with_errors > 0:
        print(f"  查询失败的节点: {nodes_with_errors}")
    print(f"  总参数数: {total_params}")
    
    # 按参数类型统计
    if total_params > 0:
        type_count = {}
        for params in all_node_params.values():
            if params:
                for param in params:
                    param_type = param.get('type', 'unknown')
                    type_count[param_type] = type_count.get(param_type, 0) + 1
        
        if type_count:
            print(f"\n  参数类型分布:")
            for param_type, count in sorted(type_count.items()):
                print(f"    {param_type}: {count} 个")
    
    print()
    
    # ========================================================================
    # 第四部分：详细参数列表（可选）
    # ========================================================================
    if total_params > 0:
        print("-" * 70)
        print("[4] 详细参数列表")
        print("-" * 70)
        
        for node_name, params in all_node_params.items():
            if params and len(params) > 0:
                print(f"\n  节点: {node_name}")
                print("  " + "-" * 66)
                for param in params:
                    value_str = format_parameter_value(param['value'], max_length=50)
                    print(f"    • {param['name']} ({param['type']}) = {value_str}")
    
    print()
    print("=" * 70)
    print("\n  测试完成!\n")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n测试被用户中断 - 正在清理...")
        sys.exit(0)
    except Exception as e:
        print(f"\n\n错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
