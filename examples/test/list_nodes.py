"""
ROS2 节点列表查询测试脚本

测试 ROS2RobotInterface 的 list_nodes() 方法，用于查询当前运行的 ROS 2 节点列表。
此测试可以在接口连接或未连接状态下运行。
"""

import time
import sys

from ros2_robot_interface import ROS2RobotInterface, ROS2RobotInterfaceConfig


def main():
    """测试 ROS2 节点列表查询功能"""
    
    print("\n" + "=" * 70)
    print(" " * 20 + "ROS2 Node List Test")
    print("=" * 70 + "\n")
    
    # ========================================================================
    # 第一部分：测试未连接状态下的节点查询
    # ========================================================================
    print("-" * 70)
    print("[1] 测试未连接状态下的节点查询")
    print("-" * 70)
    
    # 创建配置对象
    config = ROS2RobotInterfaceConfig()
    
    # 创建接口实例（不连接）
    print("  → 创建 ROS2RobotInterface 实例（未连接）...")
    interface = ROS2RobotInterface(config)
    
    # 查询节点列表（未连接状态）
    print("  → 查询节点列表...")
    try:
        nodes = interface.list_nodes()
        print(f"  ✓ 成功查询到 {len(nodes)} 个节点\n")
        
        if nodes:
            print("  节点列表（未连接状态）:")
            print("  " + "-" * 66)
            for i, node in enumerate(nodes[:10], 1):  # 只显示前10个
                print(f"  {i:2d}. {node['full_name']}")
                print(f"      名称: {node['name']}, 命名空间: {node['namespace']}")
            if len(nodes) > 10:
                print(f"  ... 还有 {len(nodes) - 10} 个节点未显示")
            print("  " + "-" * 66)
        else:
            print("  ⚠ 未发现任何节点（可能 ROS 2 未运行或节点发现服务未就绪）")
    except Exception as e:
        print(f"  ✗ 查询节点列表失败: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    print()
    
    # ========================================================================
    # 第二部分：测试连接状态下的节点查询
    # ========================================================================
    print("-" * 70)
    print("[2] 测试连接状态下的节点查询")
    print("-" * 70)
    
    print("  → 连接到 ROS 2...")
    try:
        interface.connect()
        print("  ✓ 接口连接成功!")
    except Exception as e:
        print(f"  ✗ 连接失败: {e}")
        print("  ⚠ 继续使用未连接状态进行测试...\n")
        # 即使连接失败，也可以测试未连接状态下的功能
        print("=" * 70)
        print("\n  测试完成（仅测试了未连接状态）\n")
        return 0
    
    # 等待节点发现服务就绪
    print("  → 等待节点发现服务就绪（1秒）...")
    time.sleep(1.0)
    
    # 查询节点列表（连接状态）
    print("  → 查询节点列表（连接状态）...")
    try:
        nodes_connected = interface.list_nodes()
        print(f"  ✓ 成功查询到 {len(nodes_connected)} 个节点\n")
        
        if nodes_connected:
            print("  节点列表（连接状态）:")
            print("  " + "-" * 66)
            
            # 查找接口自身的节点
            interface_node_found = False
            for node in nodes_connected:
                if 'ros2_robot_interface' in node['name'].lower():
                    interface_node_found = True
                    print(f"  ⭐ {node['full_name']} (接口自身节点)")
                    print(f"      名称: {node['name']}, 命名空间: {node['namespace']}")
                    break
            
            # 显示其他节点
            other_nodes = [n for n in nodes_connected if 'ros2_robot_interface' not in n['name'].lower()]
            print(f"\n  其他节点（共 {len(other_nodes)} 个）:")
            for i, node in enumerate(other_nodes[:15], 1):  # 显示前15个
                print(f"  {i:2d}. {node['full_name']}")
                print(f"      名称: {node['name']}, 命名空间: {node['namespace']}")
            if len(other_nodes) > 15:
                print(f"  ... 还有 {len(other_nodes) - 15} 个节点未显示")
            
            print("  " + "-" * 66)
            
            # 统计信息
            print(f"\n  统计信息:")
            print(f"    - 总节点数: {len(nodes_connected)}")
            print(f"    - 接口自身节点: {'已找到' if interface_node_found else '未找到'}")
            print(f"    - 其他节点数: {len(other_nodes)}")
            
            # 按命名空间分组统计
            namespace_count = {}
            for node in nodes_connected:
                ns = node['namespace']
                namespace_count[ns] = namespace_count.get(ns, 0) + 1
            
            if len(namespace_count) > 1:
                print(f"    - 命名空间数: {len(namespace_count)}")
                print(f"    - 命名空间分布:")
                for ns, count in sorted(namespace_count.items()):
                    print(f"        {ns}: {count} 个节点")
        else:
            print("  ⚠ 未发现任何节点（可能 ROS 2 未运行或节点发现服务未就绪）")
    except Exception as e:
        print(f"  ✗ 查询节点列表失败: {e}")
        import traceback
        traceback.print_exc()
    
    print()
    
    # ========================================================================
    # 第三部分：比较连接前后的节点数量
    # ========================================================================
    if 'nodes' in locals() and 'nodes_connected' in locals():
        print("-" * 70)
        print("[3] 比较连接前后的节点数量")
        print("-" * 70)
        print(f"  未连接状态节点数: {len(nodes)}")
        print(f"  连接状态节点数:   {len(nodes_connected)}")
        diff = len(nodes_connected) - len(nodes)
        if diff > 0:
            print(f"  差异: +{diff} 个节点（连接后新增）")
        elif diff < 0:
            print(f"  差异: {diff} 个节点（连接后减少）")
        else:
            print(f"  差异: 无变化")
        
        # 查找新增的节点
        if diff > 0:
            nodes_before_names = {n['full_name'] for n in nodes}
            nodes_after_names = {n['full_name'] for n in nodes_connected}
            new_nodes = nodes_after_names - nodes_before_names
            if new_nodes:
                print(f"\n  新增节点:")
                for node_name in sorted(new_nodes):
                    print(f"    - {node_name}")
        print()
    
    # ========================================================================
    # 第四部分：断开连接
    # ========================================================================
    print("-" * 70)
    print("[4] 断开连接")
    print("-" * 70)
    try:
        interface.disconnect()
        print("  ✓ 接口断开成功!")
    except Exception as e:
        print(f"  ⚠ 断开连接时出现错误: {e}")
    
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
