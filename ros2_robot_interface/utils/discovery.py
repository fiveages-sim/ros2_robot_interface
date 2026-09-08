"""
ROS 2 Discovery and Query Utilities

Utility functions for discovering and querying ROS 2 nodes, topics, and parameters.
These functions can be used independently or with ROS2RobotInterface.
"""

import logging
import time
from typing import Any, Dict, List, Optional

import rclpy
from rcl_interfaces.srv import ListParameters, DescribeParameters, GetParameters, SetParameters
from rcl_interfaces.msg import ParameterType, Parameter, ParameterValue
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import get_action_names_and_types
from rclpy.node import Node

logger = logging.getLogger(__name__)

# 系统内部参数列表，这些参数在查询时会被自动过滤掉
_SYSTEM_INTERNAL_PARAMS = {
    'use_sim_time',
    'start_type_description_service',
}

# 系统内部参数前缀列表，以这些前缀开头的参数也会被过滤掉
_SYSTEM_INTERNAL_PARAM_PREFIXES = {
    'qos_overrides.',
}


def _create_temp_node(name: str, namespace: str = "") -> tuple[Node, SingleThreadedExecutor]:
    """创建临时节点和 executor，便于查询/发现类操作。"""
    if not rclpy.ok():
        rclpy.init()
    temp_node = Node(name, namespace=namespace if namespace else "")
    temp_executor = SingleThreadedExecutor()
    temp_executor.add_node(temp_node)
    return temp_node, temp_executor


def discover_topics(max_attempts: int = 30) -> List[str]:
    """Discover available ROS 2 topics with simple stabilization.
    
    Args:
        max_attempts: Maximum polling attempts while waiting for topics to stabilize.
    
    Returns:
        List of topic names.
    """
    temp_node, temp_executor = _create_temp_node("ros2_robot_interface_temp")
    
    topic_names: List[str] = []
    stable_count = 0
    last_count = 0
    
    # Initial wait to let stale topics from previous connections disappear
    time.sleep(0.5)
    
    try:
        for _ in range(max_attempts):
            for _ in range(5):
                temp_executor.spin_once(timeout_sec=0.05)
            time.sleep(0.3)
            
            topic_names_and_types = temp_node.get_topic_names_and_types()
            topic_names = [name for name, _ in topic_names_and_types]
            current_count = len(topic_names)
            
            if current_count == last_count and current_count > 2:
                stable_count += 1
                if stable_count >= 3:
                    break
            else:
                stable_count = 0
            
            last_count = current_count
    finally:
        temp_executor.shutdown()
        temp_node.destroy_node()
    
    return topic_names


def discover_actions(settle_spins: int = 10) -> List[str]:
    """Discover available ROS 2 action names.

    Args:
        settle_spins: Number of executor spins to let discovery converge.

    Returns:
        List of action names.
    """
    temp_node, temp_executor = _create_temp_node("ros2_robot_interface_action_temp")
    action_names: List[str] = []
    time.sleep(0.5)
    try:
        for _ in range(max(1, settle_spins)):
            temp_executor.spin_once(timeout_sec=0.1)
        actions_and_types = get_action_names_and_types(temp_node)
        action_names = [name for name, _ in actions_and_types]
    finally:
        temp_executor.shutdown()
        temp_node.destroy_node()
    return action_names


def list_nodes(node: Optional[Node] = None, namespace: str = "") -> List[Dict[str, str]]:
    """查询当前运行的 ROS 2 节点列表。
    
    此函数可以在连接或未连接状态下使用。如果提供了节点，会使用现有节点；
    如果未提供，会创建一个临时节点来查询。
    
    Args:
        node: 可选的 ROS 2 节点实例。如果为 None，将创建临时节点。
        namespace: 命名空间，用于临时节点创建。
    
    Returns:
        List[Dict[str, str]]: 节点信息列表，每个字典包含：
            - 'name': 节点名称（不含命名空间）
            - 'namespace': 节点命名空间
            - 'full_name': 完整节点名称（命名空间 + 名称）
    
    Example:
        ```python
        # 使用现有节点查询
        nodes = list_nodes(node=my_node)
        
        # 创建临时节点查询
        nodes = list_nodes()
        for node in nodes:
            print(f"节点: {node['full_name']}")
        ```
    """
    # Ensure rclpy is initialized
    # 如果提供了节点，使用现有节点；否则创建临时节点
    use_temp_node = node is None
    
    if use_temp_node:
        temp_node, temp_executor = _create_temp_node(
            "ros2_robot_interface_node_list_temp",
            namespace=namespace
        )
        # 等待节点发现服务就绪
        time.sleep(0.5)
        for _ in range(10):
            temp_executor.spin_once(timeout_sec=0.1)
        
        query_node = temp_node
    else:
        query_node = node
    
    try:
        # 如果使用现有节点，需要确保 executor 能够处理节点发现请求
        # 在已连接状态下，节点已经在 executor 中运行，但可能需要短暂等待
        if not use_temp_node:
            # 给节点发现服务一些时间（通过短暂等待）
            time.sleep(0.1)
        
        # 获取节点名称和命名空间
        node_names_and_namespaces = query_node.get_node_names_and_namespaces()
        
        # 构建节点信息列表
        nodes = []
        for node_name, ns in node_names_and_namespaces:
            # 确保命名空间以 / 开头（如果没有）
            if ns and not ns.startswith('/'):
                ns = '/' + ns
            elif not ns:
                ns = '/'
            
            # 构建完整名称
            if ns == '/':
                full_name = '/' + node_name
            else:
                full_name = ns + '/' + node_name
            
            nodes.append({
                'name': node_name,
                'namespace': ns,
                'full_name': full_name
            })
        
        # 按完整名称排序
        nodes.sort(key=lambda x: x['full_name'])
        
        return nodes
        
    except Exception as e:
        logger.warning(f"Failed to list nodes: {e}")
        return []
    finally:
        if use_temp_node:
            temp_executor.shutdown()
            temp_node.destroy_node()


def list_node_parameters(
    full_node_name: str,
    node: Optional[Node] = None,
    namespace: str = ""
) -> List[Dict[str, Any]]:
    """查询指定节点的可动态配置参数。
    
    此函数可以在连接或未连接状态下使用。如果提供了节点，会使用现有节点；
    如果未提供，会创建一个临时节点来查询。
    
    Args:
        full_node_name: 完整节点名称（包含命名空间），例如 "/my_namespace/my_node" 或 "/my_node"
        node: 可选的 ROS 2 节点实例。如果为 None，将创建临时节点。
        namespace: 命名空间，用于临时节点创建。
    
    Returns:
        List[Dict[str, Any]]: 参数信息列表，每个字典包含：
            - 'name': 参数名称
            - 'type': 参数类型（'bool', 'int', 'double', 'string', 'byte_array', 'bool_array', 'int_array', 'double_array', 'string_array'）
            - 'value': 参数当前值
            - 'description': 参数描述（如果有）
            - 'read_only': 是否为只读参数
            - 'dynamic_typing': 是否支持动态类型
    
    Example:
        ```python
        # 使用现有节点查询
        params = list_node_parameters("/my_namespace/my_node", node=my_node)
        
        # 创建临时节点查询
        params = list_node_parameters("/my_namespace/my_node")
        for param in params:
            print(f"参数: {param['name']}")
            print(f"  类型: {param['type']}")
            print(f"  值: {param['value']}")
        ```
    """
    # Ensure rclpy is initialized
    # 如果提供了节点，使用现有节点；否则创建临时节点
    use_temp_node = node is None
    
    if use_temp_node:
        temp_node, temp_executor = _create_temp_node(
            "ros2_robot_interface_param_list_temp",
            namespace=namespace
        )
        
        # 等待节点发现服务就绪
        time.sleep(0.5)
        for _ in range(10):
            temp_executor.spin_once(timeout_sec=0.1)
        
        query_node = temp_node
    else:
        query_node = node
    
    try:
        # 确保完整节点名称格式正确（必须以 / 开头）
        if not full_node_name.startswith('/'):
            full_node_name = '/' + full_node_name
        
        # 创建参数服务客户端
        list_params_client = query_node.create_client(
            ListParameters,
            f"{full_node_name}/list_parameters"
        )
        describe_params_client = query_node.create_client(
            DescribeParameters,
            f"{full_node_name}/describe_parameters"
        )
        get_params_client = query_node.create_client(
            GetParameters,
            f"{full_node_name}/get_parameters"
        )
        
        # 等待服务可用
        if not list_params_client.wait_for_service(timeout_sec=2.0):
            logger.warning(f"Parameter service not available for node: {full_node_name}")
            return []
        
        # 调用 list_parameters 服务
        request = ListParameters.Request()
        request.depth = 0  # 0 表示只列出直接参数，不递归
        
        future = list_params_client.call_async(request)
        
        # 等待响应（需要 executor 处理）
        if use_temp_node:
            rclpy.spin_until_future_complete(temp_node, future, timeout_sec=2.0)
        else:
            # 如果使用现有节点，需要确保 executor 能够处理请求
            # 由于 executor 在后台线程运行，我们需要等待一段时间
            timeout = time.time() + 2.0
            while not future.done() and time.time() < timeout:
                time.sleep(0.1)
        
        if not future.done():
            logger.warning(f"Timeout waiting for list_parameters response from {full_node_name}")
            return []
        
        response = future.result()
        if response is None:
            logger.warning(f"Failed to get list_parameters response from {full_node_name}")
            return []
        
        param_names = response.result.names
        if not param_names:
            return []
        
        # 获取参数描述
        describe_request = DescribeParameters.Request()
        describe_request.names = param_names
        
        describe_future = describe_params_client.call_async(describe_request)
        if use_temp_node:
            rclpy.spin_until_future_complete(temp_node, describe_future, timeout_sec=2.0)
        else:
            timeout = time.time() + 2.0
            while not describe_future.done() and time.time() < timeout:
                time.sleep(0.1)
        
        describe_response = None
        if describe_future.done():
            describe_response = describe_future.result()
        
        # 获取参数值
        get_request = GetParameters.Request()
        get_request.names = param_names
        
        get_future = get_params_client.call_async(get_request)
        if use_temp_node:
            rclpy.spin_until_future_complete(temp_node, get_future, timeout_sec=2.0)
        else:
            timeout = time.time() + 2.0
            while not get_future.done() and time.time() < timeout:
                time.sleep(0.1)
        
        get_response = None
        if get_future.done():
            get_response = get_future.result()
        
        # 构建参数信息列表
        parameters = []
        type_map = {
            ParameterType.PARAMETER_NOT_SET: 'not_set',
            ParameterType.PARAMETER_BOOL: 'bool',
            ParameterType.PARAMETER_INTEGER: 'int',
            ParameterType.PARAMETER_DOUBLE: 'double',
            ParameterType.PARAMETER_STRING: 'string',
            ParameterType.PARAMETER_BYTE_ARRAY: 'byte_array',
            ParameterType.PARAMETER_BOOL_ARRAY: 'bool_array',
            ParameterType.PARAMETER_INTEGER_ARRAY: 'int_array',
            ParameterType.PARAMETER_DOUBLE_ARRAY: 'double_array',
            ParameterType.PARAMETER_STRING_ARRAY: 'string_array',
        }
        
        for i, param_name in enumerate(param_names):
            param_info = {
                'name': param_name,
                'type': 'unknown',
                'value': None,
                'description': '',
                'read_only': False,
                'dynamic_typing': False
            }
            
            # 获取参数描述
            if describe_response and i < len(describe_response.descriptors):
                descriptor = describe_response.descriptors[i]
                param_info['type'] = type_map.get(descriptor.type, 'unknown')
                param_info['description'] = descriptor.description or ''
                param_info['read_only'] = descriptor.read_only
                param_info['dynamic_typing'] = descriptor.dynamic_typing
            
            # 获取参数值
            if get_response and i < len(get_response.values):
                param_value = get_response.values[i]
                param_info['type'] = type_map.get(param_value.type, 'unknown')
                
                # 根据类型提取值
                if param_value.type == ParameterType.PARAMETER_BOOL:
                    param_info['value'] = param_value.bool_value
                elif param_value.type == ParameterType.PARAMETER_INTEGER:
                    param_info['value'] = param_value.integer_value
                elif param_value.type == ParameterType.PARAMETER_DOUBLE:
                    param_info['value'] = param_value.double_value
                elif param_value.type == ParameterType.PARAMETER_STRING:
                    param_info['value'] = param_value.string_value
                elif param_value.type == ParameterType.PARAMETER_BYTE_ARRAY:
                    param_info['value'] = list(param_value.byte_array_value)
                elif param_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
                    param_info['value'] = list(param_value.bool_array_value)
                elif param_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
                    param_info['value'] = list(param_value.integer_array_value)
                elif param_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    param_info['value'] = list(param_value.double_array_value)
                elif param_value.type == ParameterType.PARAMETER_STRING_ARRAY:
                    param_info['value'] = list(param_value.string_array_value)
            
            parameters.append(param_info)
        
        # 过滤掉系统内部参数
        def is_system_param(param_name: str) -> bool:
            """检查参数是否为系统内部参数"""
            # 检查完全匹配
            if param_name in _SYSTEM_INTERNAL_PARAMS:
                return True
            # 检查前缀匹配
            for prefix in _SYSTEM_INTERNAL_PARAM_PREFIXES:
                if param_name.startswith(prefix):
                    return True
            return False
        
        parameters = [p for p in parameters if not is_system_param(p['name'])]
        
        # 按参数名称排序
        parameters.sort(key=lambda x: x['name'])
        
        # 清理客户端
        if list_params_client:
            query_node.destroy_client(list_params_client)
        if describe_params_client:
            query_node.destroy_client(describe_params_client)
        if get_params_client:
            query_node.destroy_client(get_params_client)
        
        return parameters
        
    except Exception as e:
        logger.warning(f"Failed to list parameters for node {full_node_name}: {e}", exc_info=True)
        return []
    finally:
        if use_temp_node:
            temp_executor.shutdown()
            temp_node.destroy_node()


def set_node_parameters(
    full_node_name: str,
    parameters: Dict[str, Any],
    node: Optional[Node] = None,
    namespace: str = ""
) -> bool:
    """设置指定节点的参数值。
    
    此函数可以在连接或未连接状态下使用。如果提供了节点，会使用现有节点；
    如果未提供，会创建一个临时节点来设置参数。
    
    Args:
        full_node_name: 完整节点名称（包含命名空间），例如 "/my_namespace/my_node" 或 "/my_node"
        parameters: 参数字典，键为参数名，值为参数值
        node: 可选的 ROS 2 节点实例。如果为 None，将创建临时节点。
        namespace: 命名空间，用于临时节点创建。
    
    Returns:
        bool: 如果所有参数设置成功返回 True，否则返回 False
    
    Example:
        ```python
        # 使用现有节点设置参数
        success = set_node_parameters("/my_namespace/my_node", {"param1": 10, "param2": "value"}, node=my_node)
        
        # 创建临时节点设置参数
        success = set_node_parameters("/my_namespace/my_node", {"param1": 10, "param2": "value"})
        ```
    """
    # Ensure rclpy is initialized
    if not rclpy.ok():
        rclpy.init()
    
    # 如果提供了节点，使用现有节点；否则创建临时节点
    use_temp_node = node is None
    
    if use_temp_node:
        temp_node, temp_executor = _create_temp_node(
            "ros2_robot_interface_param_set_temp",
            namespace=namespace
        )
        
        # 等待节点发现服务就绪
        time.sleep(0.5)
        for _ in range(10):
            temp_executor.spin_once(timeout_sec=0.1)
        
        query_node = temp_node
    else:
        query_node = node
    
    try:
        # 确保完整节点名称格式正确（必须以 / 开头）
        if not full_node_name.startswith('/'):
            full_node_name = '/' + full_node_name
        
        # 创建参数服务客户端
        set_params_client = query_node.create_client(
            SetParameters,
            f"{full_node_name}/set_parameters"
        )
        
        # 等待服务可用
        if not set_params_client.wait_for_service(timeout_sec=2.0):
            logger.warning(f"Set parameter service not available for node: {full_node_name}")
            return False
        
        # 构建参数列表
        param_list = []
        for param_name, param_value in parameters.items():
            param = Parameter()
            param.name = param_name
            
            # 根据值的类型设置参数值
            if isinstance(param_value, bool):
                param.value.type = ParameterType.PARAMETER_BOOL
                param.value.bool_value = param_value
            elif isinstance(param_value, int):
                param.value.type = ParameterType.PARAMETER_INTEGER
                param.value.integer_value = param_value
            elif isinstance(param_value, float):
                param.value.type = ParameterType.PARAMETER_DOUBLE
                param.value.double_value = param_value
            elif isinstance(param_value, str):
                param.value.type = ParameterType.PARAMETER_STRING
                param.value.string_value = param_value
            elif isinstance(param_value, (list, tuple)):
                # 尝试推断数组类型
                if len(param_value) == 0:
                    # 空数组，默认使用整数数组
                    param.value.type = ParameterType.PARAMETER_INTEGER_ARRAY
                    param.value.integer_array_value = []
                elif isinstance(param_value[0], bool):
                    param.value.type = ParameterType.PARAMETER_BOOL_ARRAY
                    param.value.bool_array_value = list(param_value)
                elif isinstance(param_value[0], int):
                    param.value.type = ParameterType.PARAMETER_INTEGER_ARRAY
                    param.value.integer_array_value = list(param_value)
                elif isinstance(param_value[0], float):
                    param.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                    param.value.double_array_value = list(param_value)
                elif isinstance(param_value[0], str):
                    param.value.type = ParameterType.PARAMETER_STRING_ARRAY
                    param.value.string_array_value = list(param_value)
                else:
                    logger.warning(f"Unsupported array element type for parameter {param_name}, using string array")
                    param.value.type = ParameterType.PARAMETER_STRING_ARRAY
                    param.value.string_array_value = [str(v) for v in param_value]
            else:
                logger.warning(f"Unsupported parameter type for {param_name}: {type(param_value)}, converting to string")
                param.value.type = ParameterType.PARAMETER_STRING
                param.value.string_value = str(param_value)
            
            param_list.append(param)
        
        # 调用 set_parameters 服务
        request = SetParameters.Request()
        request.parameters = param_list
        
        future = set_params_client.call_async(request)
        
        # 等待响应
        if use_temp_node:
            rclpy.spin_until_future_complete(temp_node, future, timeout_sec=2.0)
        else:
            timeout = time.time() + 2.0
            while not future.done() and time.time() < timeout:
                time.sleep(0.1)
        
        if not future.done():
            logger.warning(f"Timeout waiting for set_parameters response from {full_node_name}")
            return False
        
        response = future.result()
        if response is None:
            logger.warning(f"Failed to get set_parameters response from {full_node_name}")
            return False
        
        # 检查结果
        all_success = True
        for i, result in enumerate(response.results):
            if not result.successful:
                param_name = param_list[i].name
                logger.warning(f"Failed to set parameter {param_name}: {result.reason}")
                all_success = False
        
        # 清理客户端
        if set_params_client:
            query_node.destroy_client(set_params_client)
        
        return all_success
        
    except Exception as e:
        logger.warning(f"Failed to set parameters for node {full_node_name}: {e}", exc_info=True)
        return False
    finally:
        if use_temp_node:
            temp_executor.shutdown()
            temp_node.destroy_node()
