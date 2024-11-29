#!/usr/bin/env python
import csv
import argparse
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

    def __init__(self):
        super().__init__('code_gen_client')
        self.group = MutuallyExclusiveCallbackGroup()
        self.cli = self.create_client(CodeGen, 'code_gen', callback_group=self.group)
        self.req = CodeGen.Request()
        self.get_logger().info("Client ready!")
    
    def generate_inference(self, prompt: str, preamble = "", policies=""):
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

def read_inputs(input_path):
    inputs = []

    with open(input_path, 'r') as input_file:
        inputs = input_file.read().split('\n')

    return inputs

def write_outputs(output_path, outputs):

        with open(output_path, 'w') as output_file:
            writer = csv.writer(output_file)
            for row in outputs:
                writer.writerow(row)

def write_to_log(log_path, entry, erase=False):

    with open(log_path, 'a') as log_file:
        if erase:
            log_file.truncate(0)
        log_file.write('[' + str(time.time()) + ']: ')
        log_file.write(entry)

def test(codegen, inputs, log_path):

    # Prepare code generation
    preamble = ""
    preamble_path = Path(__file__).with_name("preamble.txt")
    with open(preamble_path, 'r') as preamble_file:
        preamble = preamble_file.read()

    policy_bank = PolicyBank()

    preamble = preamble + policy_bank.imports + policy_bank.hints

    data = []
    # Take measurements
    for i in range(len(inputs)):
        start = time.time()
        result = codegen.generate_inference(inputs[i], preamble, policy_bank.definitions)
        end = time.time()

        data.append(end-start)
        entry = inputs[i] + '' + result + '\n'
        write_to_log(log_path, entry)
        time.sleep(0.5)
    
    return data


def main():

    # Get necessary files as arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('input_path')
    parser.add_argument('output_path')
    parser.add_argument('-i', '--iterations', help="number of tests to run (optional)")
    parser.add_argument('-l', '--log', help="log file (optional)")
    args = parser.parse_args()

    inputs = read_inputs(args.input_path)
    outputs = [[] for i in range(len(inputs))]
    n = int(args.iterations) if args.iterations is not None else 1
    log_path = args.log if args.log is not None else 'log.txt'

    # ROS2 setup
    rclpy.init()
    executor = MultiThreadedExecutor()
    codegen = CodeGenClient()
    executor.add_node(codegen)
    et = threading.Thread(target=executor.spin)
    et.start()

    write_to_log(log_path, "Starting tests!\n", erase=True)

    # Run tests
    for i in range(n):
        write_to_log(log_path, "Test #" +  str(i) + "\n")
        data = test(codegen, inputs, log_path)
        # Concatenate data to outputs
        outputs = [outputs[i] + [data[i]] for i in range(len(inputs))]

    write_to_log(log_path, "Writing outputs!\n")
    write_outputs(args.output_path, outputs)

    executor.shutdown()
    codegen.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()