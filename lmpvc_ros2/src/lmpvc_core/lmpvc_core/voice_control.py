#!/usr/bin/env python

# This is the main module of the LMPVC suite

import ast
import astunparse
import json
import threading
import traceback
import sys
import os
from pathlib import Path

from lmpvc_core.policy_bank import PolicyBank
from lmpvc_core.codegen_cli import CodeGenClient as CodeGen
from lmpvc_core.robot import RobotAPI as Robot
from lmpvc_core.listener_cli import ListenerActionClient as Listener

def empty_fn(*args, **kwargs):
    """Dummy function used to replace unsafe function calls"""
    pass

def merge_dicts(dicts):
    return {
        k : v 
        for d in dicts
        for k, v in d.items()
    }

def exec_safe(code, globals):
    """Final check before generated code is executred"""

    banned_phrases = ['__', 'robot.enable', 'robot.enabled']
    for phrase in banned_phrases:
        if phrase in code:
            raise Exception("Code contained banned phrase: " + phrase)

    custom_globals = merge_dicts([globals, {'exec': empty_fn, 'eval': empty_fn}])
    exec(code, custom_globals)

class ExecThread(threading.Thread):
    """Thread to allow non-blocking execution of robot code"""

    def __init__(self, robot, policy_bank):
        threading.Thread.__init__(self)
        self.robot = robot
        self.policy_bank = policy_bank
        self.proceed = threading.Event()
        self.lock = threading.Lock()
        self.queue = []
        self.reset = False
        self.quit = False
        self.stopped = False
        self.e = None

    def execute(self, code):
        """Implements a single execution sequence"""
        import numpy as np
        import math
        from scipy.spatial.transform import Rotation as R
        robot = self.robot

        code_with_policies = '\n'.join(self.policy_bank._imports[:-1]) + self.policy_bank.definitions + code

        try:
            exec_safe(code_with_policies, locals())
            return True
        
        except Exception as e:
            self.robot.stop()
            #print(traceback.format_exc())
            robot.say("Execution failed: " + str(e))
            return False
        
    def run(self):
        """Thread main loop"""
        
        code_queue = []
        no_exceptions = True
        quit = False

        try:

            while(True):

                # Wait for main thread to allow execution
                while(not self.proceed.wait(.05)):
                    with self.lock:
                        if self.quit:
                            return

                # Check if code queue has been reset by main thread
                with self.lock:
                    if self.reset:
                            self.reset = False
                            code_queue = self.queue
                            no_exceptions = True
                
                # Execute from queue until an exception occurs
                if len(code_queue) > 0 and no_exceptions:
                    if self.execute(code_queue[0]):
                        code_queue.pop(0)
                    else:
                        no_exceptions = False

        except Exception as e:

            # Stores the exception and sets a flag fot the main thread
            with self.lock:
                self.e = e
                self.stopped = True

class VoiceControl:
    """This is the main control loop of the entire LMPVC suite"""

    def __init__(self, codegen, robot, listener):

        try:
            self.model = codegen
            self.robot = robot
            self.listener = listener
            self.policy_bank = PolicyBank()
            self._exec_thread = ExecThread(self.robot, self.policy_bank)

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
            
            self.text_only = config['text_only']

            # Get the preamble text containing hints for the generator
            self.preamble = ""
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
                self.preamble = preamble_file.read()
            
            self.manual_review = config['manual_review']

            # Defines a context window for remembering previous prompt + code pairs
            self.context_window = []
            self.context_length = config['context_lenght']

            self.robot.stop(hard=True)
            self._exec_thread.start()

            self._record = False
            self._memory = []
        
        except KeyboardInterrupt:
            print("Quitting Voice Control: Keyboard interrupt")
            raise

        except Exception as e:
            print("Quitting Voice Control: Exception encountered during setup:", e)
            raise

    def overrides(self, command):
        """
        Handles special keyword commands quickly
        Return: True if a keyword was found, False otherwise
        """

        self.robot.enable()

        if command == 'stop':
            print("\nAttempting to stop robot!")
            self.robot.stop(hard = True)
            self.robot.say("stopping")
            return True
        
        elif command == 'set home':
            self.robot.set_home()
            return True
        
        elif command == 'set limits':
            self.robot.set_limits()
            return True
        
        elif command == 'record policy':
            self.robot.say("Recording!")
            self._record = True
            return True
        
        elif command == 'discard policy':
            self.robot.say("Discarding!")
            self._record = False
            self._memory = []
            return True
        
        elif command == 'save policy':
            self.robot.say("Saving!")
            self.robot.say("Please type details.")
            name = input("\nName of the policy(e.g. jiggling_sequence): ")
            hint = input("\nHint for the LLM (e.g. please jiggle): ")
            self._record = False
            if self.policy_bank.add_policy(name, hint, self._memory):
                self.robot.say("Saving succeeded!")
            else:
                self.robot.say("Saving failed!")
            
            self._memory = []
            return True

        else:
            return False

    def generate_inference(self, command):
        """Uses the generator module to generate policy code"""
        preamble = self.preamble + self.policy_bank.imports + self.policy_bank.hints + '\n\n' + '\n\n'.join(self.context_window)

        #print(preamble)

        generated = self.model.generate_inference(command, preamble, self.policy_bank.definitions)

        return generated

    def validate(self, code, manual=False):
        """Implements safety checks before running code"""

        # Temporary safety system allows the user to inspect code before execution
        print('\n' + code)

        while manual:
                run = input("\nExecute the code? (y/n): ")

                if run == 'y':
                    return True
                elif run == 'n':
                    print("Not completed: User cancelled execution.")
                    return False
        
        return True
    
    def update_context(self, prompt, code):
        """Maintain the N(=self.cotenxt_length) latest prompt + code combinations as context"""

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
        
        context = "# define function: " + prompt + "\n" + context.strip() + "\n# end of function"

        # Avoid overly long context which might confuse the LLM
        if(len(code) <= 500):
            self.context_window.append(context)
            print("\nAppending context:")
            print(context)
        else:
            print("Context cleared, input too large!")
            self.context_window = []

        if len(self.context_window) > self.context_length:
            self.context_window.pop(0)

    def execute(self, code):
        """Interacts with _exec_thread to begin a new execution sequence"""
        code_queue = [code]

        with self._exec_thread.lock:
            # Check for exceptions in the execution thread          
            if self._exec_thread.stopped:
                raise self._exec_thread.e
                
            self._exec_thread.reset = True
            self._exec_thread.queue = code_queue

        # Enable robot to move and unblock the execution thread    
        self.robot.enable()
        self._exec_thread.proceed.set()   

    def stop(self):
        """Implements a safety routine at whenever the program quits for any reason"""

        print("\nAttempting to stop robot!")

        # Block the execution thread and hard stop the robot
        self._exec_thread.proceed.clear()
        self.robot.stop(hard=True)

        # Signal the execution thread to stop looping
        with self._exec_thread.lock:
            self._exec_thread.quit = True

        self._exec_thread.join()

    def interface(self):
        """The interface and main loop of the voice control module"""

        try:
            self.robot.say("Voice Control activated!", wait=False)
            #self.robot.set_home()
            command = ''
            code = ''

            # Start the UI
            while(True):

                # Check for exceptions in the execution thread 
                with self._exec_thread.lock:                      
                    if self._exec_thread.stopped:
                        raise self._exec_thread.e

                if self.text_only:
                    command = input("\nInstruction: ")
                else:
                    input("\nPress any key to begin listening")
                    command = self.listener.listen()
                    print("\nInstruction:", command)
                
                command = command.strip()

                if command == 'timeout':
                    print('Input timed out, restarting!')
                    continue

                # Block the execution thread and hard stop the robot
                self._exec_thread.proceed.clear()
                self.robot.stop(hard=True)

                # Manually intercept certain commands for fast response
                if command == 'quit' or command == 'exit':
                    break
                
                if self.overrides(command):
                    print("\nDetected a pre-defined keyword!")
                    continue

                # Get inference from the generator implementation
                print("\nGenerating inference...")
                code = self.generate_inference(command)
                
                # Hand over generated code for execution
                print("\nAttempting to execute generated policy:")

                if self.validate(code, manual=self.manual_review):
                    if self.context_length > 0:
                        self.update_context(command, code)

                    self.execute(code)

                    if self._record:
                        print("\nRecorded to memory!")
                        self._memory.append(code)
            
            self.stop()
            print("\nQuitting Voice Control: User command")
        
        except KeyboardInterrupt:
            # Stop the robot in the event of a keyboard interrupt by the user
            self.stop()
            print("\nQuitting Voice Control: Keyboard interrupt")
            raise

        except Exception as e:
            # Stop the robot in the event of any exception during main loop
            self.stop()
            print("\nQuitting Voice Control: Exception encountered during operation: ", e)
            raise



def start_ui():

    try:

        config_path = Path(__file__).with_name('preamble.txt')

        vc = VoiceControl(config_path, text_only=True)

        vc.interface()

        print("\nDone.")

    except KeyboardInterrupt:
        print("\nDone.")
    except IOError:
        print("\nCouldn't open config file! ", config_path)


if __name__ == '__main__':
    start_ui()