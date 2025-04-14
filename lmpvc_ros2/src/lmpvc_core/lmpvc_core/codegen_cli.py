#!/usr/bin/env python
import ast
import astunparse
import json
import threading
import time
import rclpy
import os

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from pathlib import Path
from lmpvc_interfaces.srv import CodeGen

from lmpvc_core.policy_bank import PolicyBank

class CodeGenClient:
    """ROS2 client node for calling lmpvc_codegen."""

    def __init__(self, node):
        self.node = node
        self.group = MutuallyExclusiveCallbackGroup()
        self.cli = self.node.create_client(CodeGen, 'code_gen', callback_group=self.group)
        self.req = CodeGen.Request()
        self.node.get_logger().info("Client ready!")
    
    def generate_inference(self, prompt: str, preamble = "", policies=""):
        """Service to call the LLM. Cut down but compatible client version of the
            generate_inference implemented internally bycodegen.py.
        """
        self.req.prompt = prompt
        self.req.preamble = preamble
        self.req.policies = policies

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Service not available, trying again...")

        self.node.get_logger().info("Sending request...")
        start = time.time()
        result = self.cli.call(self.req)
        end = time.time()

        self.node.get_logger().info("Response received in " + str(end - start) + " seconds!")
        return result.result


def create_context(prompt, code):
    """Used to create simple context for testing"""
    f_defs = []
        
    # Find function definitions as AST nodes
    for node in ast.parse(code).body:
        if isinstance(node, ast.FunctionDef):
            f_defs.append(node)

    f_srcs = []

    # Unparse the AST nodes back to source code
    for f_def in f_defs:
        f_srcs.append(astunparse.unparse(f_def))

    context = '\n\n'.join(f_srcs)
    
    context = "\n\n# define function: " + prompt + "\n" + context.strip() + "\n# end of function"

    return context

def test(codegen):
    """Replicates the basic behaviour of voice_control.py for testing."""
    config_path = ""
            
    try:
        from ament_index_python.packages import get_package_share_directory

        config_path = os.path.join(
            get_package_share_directory('lmpvc_core'),
            'core_config.json'
        )
    except:
        config_path = Path(__file__).with_name('core_config.json')
    config = {}

    with open(config_path, 'r') as config_file:
        config = json.load(config_file)['voice_control']

    # Get the preamble text containing hints for the generator
    preamble = ""
    preamble_path = ""
    
    try:
        from ament_index_python.packages import get_package_share_directory

        preamble_path = os.path.join(
            get_package_share_directory('lmpvc_core'),
            'preamble.py'
        )
    except:
        preamble_path = Path(__file__).with_name('preamble.py')

    with open(preamble_path, 'r') as preamble_file:
        preamble = preamble_file.read()

    policy_bank = PolicyBank()

    preamble = preamble + policy_bank.imports + policy_bank.hints

    context = ""

    while True:
        cmd = input("\nInstruction (q to quit): ")

        if cmd == 'q':
            break
        
        result = codegen.generate_inference(cmd, preamble + context, policy_bank.definitions)
        print("Result:\n")
        print(result)

        if config['context_lenght'] > 0:
            context = create_context(cmd, result)


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = Node('codegen_client')
    codegen = CodeGenClient(node)
    
    executor.add_node(node)
    et = threading.Thread(target=executor.spin)
    et.start()

    test(codegen)

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()