#!/usr/bin/env python

import ast
import astunparse
import inspect
import json
import os
from pathlib import Path

class PolicyBank:
    """Saves and loads policies to and from files.
        Known policies are defined in index.json.
    """
    def __init__(self):
        """Read configuration and load the bank to memory"""
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
            config = json.load(config_file)['policy_bank']
        
        self._dir = str(Path(__file__).with_name(config['sub_directory']))
        self._blacklist = config['blacklist']
        self._hint_tag = config['hint_tag']

        self._index_path = self._dir + '/index.json'
        self._index = {}
        self._hints = {}
        self._srcs = {}
        self._imports = []

        self.imports = ""
        self.hints = ""
        self.definitions = ""

        self.load_from_disk()
        self.update()

        #print(self._index)
        #print(self._hints)
        #print(self._srcs)

    def load_from_disk(self):
        """Loads the index file and corresponding sources from disk"""
        # Update index
        with open(self._index_path, 'r') as index_file:
            self._index = json.load(index_file)
        # Load existing policies into memory
        for policy, src_path in self._index.items():

            if policy in self._blacklist:
                continue

            with open(self._dir + '/' + src_path, 'r') as src_file:
                src = src_file.read()
                hint = None

                if '# BODY' in src:
                    parts = src.split('# BODY')
                    imports = [i for i in parts[0].split('\n') if i != '']
                    for i in imports:
                        if i not in self._imports:
                            self._imports.append(i)
                    src = parts[1]

                if self._hint_tag in src:
                    parts = src.split(self._hint_tag)
                    src = parts[0]
                    hint = parts[1].strip()

                self._srcs.update({policy: src.strip()})
                self._hints.update({policy: hint})
        
        self._imports.append("from policies import " + ', '.join(self._srcs.keys()))

    
    def save_to_disk(self):
        """Saves the index and corresponding sources to disk"""
        # Update index on disk
        with open(self._index_path, 'w') as index_file:
            json.dump(self._index, index_file, indent=4)
        # Write sources to paths specified in the index
        for policy, src_path in self._index.items():
            if Path(self._dir + '/' + src_path).exists() or policy in self._blacklist:
                continue
            with open(self._dir + '/' + src_path, 'w') as src_file:
                src_file.write(self._srcs[policy])
                if self._hints[policy] is not None:
                    src_file.write('\n' + self._hint_tag + '\n' + self._hints[policy])

    def add_simple_policy(self, name, src, prompt):
        """Saves a single generation results as a policy with a specified name
            params:
                name: the name of the new policy, doesn't have to match existing functions
                src: source code of the new policy
                prompt: the prompt used do generate the policy, this will be used to create a hint
        """

        if name in self._srcs.keys() or name in self._blacklist:
            print("\nCould not add policy: " + name + "\nPolicy already exists, and/or is blacklisted.")
            return False
        
        f_defs = []

        # Find function definitions as AST nodes
        for node in ast.parse(src).body:
            if isinstance(node, ast.FunctionDef):
                f_defs.append(node)
        
        # Change the name of the last function in the file
        # (this should be the main function of the policy)
        f_defs[-1].name = name
        f_srcs = []

        # Unparse the AST nodes back to source code
        for f_def in f_defs:
            f_srcs.append(astunparse.unparse(f_def))
        
        # Create a function object for the main function
        # and create a signature with inspect
        env = {}
        exec(f_srcs[-1], env)
        f = env[name]
        params = str(inspect.signature(f))
        f_call = name + params

        # Add the edited policy to memory
        self._index.update({name: 'src/' + name + '.py'})
        self._hints.update({name: prompt.strip() + '\n' + f_call})
        self._srcs.update({name: '\n\n'.join(f_srcs).strip()})
        self._imports[-1] = "from policies import " + ', '.join(self._srcs.keys())

        self.save_to_disk()
        self.update()

        print("\nAdded policy: " + name)
        return True
    
    def add_policy(self, name, hint, srcs):
        """Constructs a policy from multiple generation results and saves it to disk.
            Requires a pre-defined hint corresponding to the intent of the full policy.
        
            params:
                name: the name of the new policy, doesn't have to match existing functions
                srcs: list of source codes generated from a series of commands
                hint: hint to let the LLM know how to use the combined policy
        """

        if name in self._srcs.keys() or name in self._blacklist:
            print("\nCould not add policy: " + name + "\nPolicy already exists, and/or is blacklisted.")
            return False
        
        src = '\n\n'.join(srcs)

        f_srcs = []
        # Find function definitions
        for node in ast.parse(src).body:
            if isinstance(node, ast.FunctionDef):
                f_srcs.append(astunparse.unparse(node).strip())

        f_calls = []
        # Find function calls
        for node in ast.parse(src).body:
            if isinstance(node, ast.Expr):
                if isinstance(node.value, ast.Call):
                    f_calls.append(astunparse.unparse(node).strip())
        
        f_main_params = []
        # Collect all unique arguments
        for f_call in f_calls:
            params = f_call.split(',')
            params[0] = params[0].split('(')[1]
            params[-1] = params[-1].split(')')[0]

            for param in params:
                param = param.strip()
                if param != '' and param not in f_main_params:
                    f_main_params.append(param)

        # Construct a call for the entire policy
        f_main_call = name + '(' + ', '.join(f_main_params) + ')'
        # Construct a function which executes the call
        f_main = 'def ' + f_main_call + ':\n'
        for f_call in f_calls:
            f_main += '    ' + f_call + '\n'
        
        f_srcs.append(f_main.strip())

        print('\n\n'.join(f_srcs) + '\n\n' + f_main_call)

        f_hint = 'def ' + hint.replace(' ', '_') + '(' + ', '.join(f_main_params) + '):'

        # Add the edited policy to memory
        self._index.update({name: 'src/' + name + '.py'})
        self._hints.update({name: '# define function: ' + hint  + '\n' + f_hint +
                            '\n    ' + f_main_call + '\n# end of function'})
        self._srcs.update({name: '\n\n'.join(f_srcs).strip()})
        self._imports[-1] = "from policies import " + ', '.join(self._srcs.keys())

        self.save_to_disk()
        self.update()

        print("\nAdded policy: " + name)
        return True
    
    def update(self):
        """Updates the imports, hints and definitions strings used for generation
            and execution
        """
        #print("Updating!")

        if len(self._srcs.keys()) > 0:
            self.imports = '\n\n' + '\n'.join(self._imports) + '\n\n'
            self.hints = '\n\n' + '\n\n'.join(list(self._hints.values())) + '\n\n'
            self.definitions = '\n\n' + '\n\n'.join(list(self._srcs.values())) + '\n\n'
            
            #print("POLICY BANK")
            #print("Imports:")
            #print('\n' + self.imports)
            #print("Hints:")
            #print(self.hints)
            #print("Definitions:")
            #print(self.definitions)
    

def main():
    bank = PolicyBank()
    print(bank.hints)
    #bank.add_policy('hello_and_bye', 'say hello and bye', ["def hello(robot):\n    print('Hello World!')\nhello(robot)\n",
                                            #"def bye(robot, door):\n    print('Goodbye Cruel World!')\nbye(robot, door)\n"])
    #bank.combine_policies('hello_encapsulated', ["def hello():\n    print('Hello World!')\nhello()\n"])
    #bank.add_policy('hello', "def hello():\n    print('Hello World!')\nhello()\n", '# say hello\n')
    #bank.add_policy('goodbye', "def bye():\n    print('Goodbye Cruel World!')\nbye()\n", '# say goodbye\n')

if __name__ == '__main__':
    main()