#!/usr/bin/env python
import ast
import astunparse
import json
import threading
import time
import rclpy

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from pathlib import Path
from lmpvc_interfaces.srv import CodeGen

from lmpvc_core.policy_bank import PolicyBank

class CodeGenClient(Node):
    """ROS2 client node for calling lmpvc_codegen."""

    def __init__(self):
        super().__init__('code_gen_client')
        self.group = MutuallyExclusiveCallbackGroup()
        self.cli = self.create_client(CodeGen, 'code_gen', callback_group=self.group)
        self.req = CodeGen.Request()
        self.get_logger().info("Client ready!")
    
    def generate_inference(self, prompt: str, preamble = "", policies=""):
        """Service to call the LLM. Cut down but compatible client version of the
            generate_inference implemented internally bycodegen.py.
        """
        self.req.prompt = prompt
        self.req.preamble = preamble
        self.req.policies = policies

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, trying again...")

        self.get_logger().info("Sending request...")
        start = time.time()
        result = self.cli.call(self.req)
        end = time.time()

        self.get_logger().info("Response received in " + str(end - start) + " seconds!")
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
    config_path = Path(__file__).with_name('core_config.json')
    config = {}

    with open(config_path, 'r') as config_file:
        config = json.load(config_file)['voice_control']

    preamble = ""
    preamble_path = Path(__file__).with_name(config['preamble_file'])

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
    codegen = CodeGenClient()
    executor.add_node(codegen)
    et = threading.Thread(target=executor.spin)
    et.start()

    test(codegen)

    executor.shutdown()
    codegen.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()