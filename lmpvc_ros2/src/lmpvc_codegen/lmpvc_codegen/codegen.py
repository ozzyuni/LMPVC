#!/usr/bin/env python
#
# This module implements hierarchical code generation on top of a variety
# of code generating LLMs available in this package. Some of it is based on
# modified code from Code as Policies:
#
# https://github.com/google-research/google-research/blob/master/code_as_policies/Experiment_%20HumanEval%20Benchmark.ipynb
#
# ORIGINAL LICENSE:
#
# Copyright 2022 Google LLC.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ast
import astunparse
import inspect
import json
import os
import time
from pathlib import Path

from lmpvc_codegen.codegen_cache import CodeGenCache

try:
    from ament_index_python.packages import get_package_share_directory

    codegen_config_path = os.path.join(
        get_package_share_directory('lmpvc_codegen'),
        'codegen_config.json'
    )
except:
    codegen_config_path = Path(__file__).with_name('codegen_config.json')

codegen_config = {}
with open(codegen_config_path, 'r') as config_file:
    codegen_config = json.load(config_file)

# Importing the correct file based on configuration
if codegen_config['model'] == 'bitsandbytes' or codegen_config['model'] == 'gptq':
    from lmpvc_codegen.starcoder2_local import Model
elif codegen_config['model'] == 'gguf':
    from lmpvc_codegen.starcoder2_gguf import Model
else:
    if codegen_config['model'] != 'inference_api':
        print("Invalid parameter 'model', defaulting to inference_api!")
    from lmpvc_codegen.starcoder2_inference_api import Model


class FunctionFinder(ast.NodeVisitor):
    """Finds function calls and assignments in an AST node."""
    
    def __init__(self):
        """Init FunctionFinder
        _f_calls:    a dict which will be formatted as f_calls[f_name] = 'f_name(x)'
        _f_assigns:  a dict which will be formatted as f_assigns[f_name] = 'y = f_name(x)'
        """
        super().__init__()
        self._f_calls = {}
        self._f_assigns = {}
    
    def visit_Call(self, node):
        self.generic_visit(node)

        if isinstance(node.func, ast.Name):
            f_call = astunparse.unparse(node).strip()
            f_name = astunparse.unparse(node.func).strip()
            self._f_calls[f_name] = f_call
        
        return node
    
    def visit_Assign(self, node):
        self.generic_visit(node)

        if isinstance(node.value, ast.Call):
            f_assign = astunparse.unparse(node).strip()
            f_name = astunparse.unparse(node.value.func).strip()
            self._f_assigns[f_name] = f_assign
        
        return node
    
    def search(self, node):
        self.visit(node)
        return self._f_calls, self._f_assigns

def merge_dicts(dicts):
    return {
        k : v 
        for d in dicts
        for k, v in d.items()
    }


def in_scope(f_name, all_vars):
    """Checks if function f_name is included in the scope of all_vars"""

    exists = True
    
    try:
        eval(f_name, all_vars)
    except:
        exists = False

    return exists

def empty_fn(*args, **kwargs):
    pass

def exec_safe(code_str, gvars, lvars):

    banned_phrases = ['import', '__']
    for phrase in banned_phrases:
        if phrase in code_str:
            raise Exception('Code contained banned phrase: ' + phrase)

    custom_gvars = merge_dicts([gvars, {'exec': empty_fn, 'eval': empty_fn}])
    exec(code_str, custom_gvars, lvars)

def split_code(code, delimiter):
    """Splits a code string and includes the delimiter at the front of each block"""
    split = code.split(delimiter)
    return [delimiter + block for block in split[:-1]] + [split[-1]]


class CodeGen(object):
    """Implements hierarchical generation on top of the Model class.

        CodeGen.generate_inference follows the same syntax as Model.generate_inference,
        with optional arguments for more detailed usage.

        WARNING: This generation method only works if prompted to generate function
        definitions. Be careful if tweaking any options.
    """

    def __init__(self):
            super().__init__()

            try:
                from ament_index_python.packages import get_package_share_directory

                config_path = os.path.join(
                    get_package_share_directory('lmpvc_codegen'),
                    'codegen_config.json'
                )
            except:
                config_path = Path(__file__).with_name('codegen_config.json')

            config = {}
            with open(config_path, 'r') as config_file:
                config = json.load(config_file)

            self.model = Model()
            self.cache = CodeGenCache(filename=config['cache']['filename'])
            self.cache_enabled = config['cache']['enabled']

    def generate_lmp(self, prompt, preamble = '', template=('# define function: ', '\ndef '),
                     f_name = None, context_vars = {}, recursions_remaining = 10, log = False,
                     stop_tokens=['# end of function']):
        """Hierarchically generates program code using the provided model"""

        f = None
        fs = {}
        f_srcs = {}
        child_fs = {}

        gvars = context_vars
        lvars = {}

        prompt = template[0] + prompt + template[1]
        if f_name is not None:
            prompt = prompt + f_name + '('

        # Search cache
        if self.cache_enabled:
            f_gen = self.cache.search(prompt)
        else:
            f_gen = None
        # If cache search produced no result, call generation
        if f_gen is None:
            cached = False
            print("\nNo matches in cache, generating!")
            f_gen = prompt + self.model.generate_inference(prompt, preamble,
                                                           stop_tokens=stop_tokens,
                                                           log=log)
            # Uncomment for CodeLlama!
            f_gen = f_gen.replace('<s> ', '')
        else:
            cached = True
            print("\nFound cached result!")

        f_src = ''
        f_success = True
        
        try:

            # Try to find the definition for f_name in the generated code
            if log:
                    print("\nSearching generated block:")
                    print(f_gen)

            for node in ast.parse(f_gen).body:
                if isinstance(node, ast.FunctionDef):
                    # If found, quit looking
                    if node.name == f_name:
                        f_src = astunparse.unparse(node)
                        break
                    # If f_name isn't known (=first iteration), default to the first definition found
                    elif f_name is None:
                        f_name = node.name
                        f_src = astunparse.unparse(node)
                        break

            # Create a function object for f
            exec_safe(f_src, gvars, lvars)
            f = lvars[f_name]

        except Exception as e:
            # Catch syntax errors etc. in f_src
            # f_src is only the function definition, which means any missing
            # submodules do not cause an error here
            print("Generator error: ", e, " in generated source.")
            f = empty_fn  
            f_success = False

        if log:
            print("Prompt:", prompt)
            #print('Generated Result:')
            #print(f_gen)
            print("Isolated Source:")
            print(f_src)
            print()
        
        #print(f_srcs)

        f_def_body = None

        if f_success:
            if not cached:
                self.cache.add(prompt, f_src, write_to_disk=False)

            for node in ast.parse(f_src).body:
                if isinstance(node, ast.FunctionDef):
                    f_def_body = astunparse.unparse(node.body)
                    break
        
        all_child_fs, all_child_f_srcs = {}, {}
        
        if recursions_remaining > 0 and f_def_body is not None:
        
            # Find potential child function calls and assignment       
            (potential_child_fs,  potential_child_assigns) = FunctionFinder().search(ast.parse(f_def_body))

            # For functions which are used in an assignment (-> should have a return value), replace
            # plain f(x) calls with the full y=f(x) string for better generation results
            for potential_child_f_name, potential_child_signature in potential_child_assigns.items():
                if potential_child_f_name in potential_child_fs:
                    potential_child_fs[potential_child_f_name] = potential_child_signature

            for child_f_name, child_f_signature in potential_child_fs.items():
                all_vars = merge_dicts([context_vars, all_child_fs, lvars])

                # Check for missing submodules, and generate them if necessary    
                if not in_scope(child_f_name, all_vars):
                    child_fs, child_f_srcs, child_f_success = self.generate_lmp(child_f_signature, 
                                                               preamble=preamble, 
                                                               f_name=child_f_name,
                                                               context_vars=all_vars, 
                                                               recursions_remaining=recursions_remaining-1,
                                                               log=log)

                    all_child_fs.update(child_fs)
                    all_child_f_srcs.update(child_f_srcs)
                    if not child_f_success:
                        f_success = False

                # If new child functions were generated, add them to the context for f
                if len(all_child_fs) > 0:
                    gvars = merge_dicts([context_vars, all_child_fs])
                    lvars = {}

                    exec(f_src, gvars, lvars)
                    f = lvars[f_name]
            
        elif f_def_body is None:
            print("Generation error: Parent function body is None.")
        elif recursions_remaining == 0:
            print("Generation error: Recursion limit reached!")
        
        fs = {f_name: f}
        fs.update(all_child_fs)

        f_srcs = {f_name: f_src}
        f_srcs.update(all_child_f_srcs)


        return fs, f_srcs, f_success

    def generate_inference(self, prompt, preamble='', policies='', template=('# define function: ', '\n'),
                           executable=True, log=False, context={}):
            """Uses hierarchical generation to construct the final inference output"""

            lmp = ''
            fs = {}
            srcs = {}
            success = False

            context_vars = context

            if policies != '':
                exec(policies, context_vars)

            start = time.time()
            (fs, srcs, success) = self.generate_lmp(prompt, preamble, template=template, log=log, context_vars=context_vars)
            end = time.time()

            print("Got result in", end - start, "seconds!")

            if success:
                print("\nGeneration successful, saving cache!")
                if self.cache_enabled:
                    self.cache.save()
                # Compose LMP source in a readable manner
                lmp = '\n'.join(reversed(srcs.values()))
                f_name = next(iter(srcs))
                params = str(inspect.signature(fs[f_name]))

                # Construct a function call f(x, y ...) for parent function f to run the LMP with exec()
                # Input parameter names must be controlled with prompt engineering
                if executable:
                    lmp += '\n' + f_name + params
            
            else:
                print("\nGeneration failed, reverting changes to cache!")
                if self.cache_enabled:
                    self.cache.load()

            return lmp
    
def robotest():
    preamble = ""
    preamble_path = Path(__file__).with_name('preamble.txt')

    with preamble_path.open('r') as preamble_file:
        preamble = preamble_file.read()

    model = CodeGen()
    while(True):
        command = input("\nInstruction: ")
        
        if command == 'q' or command == 'quit':
            break

        prompt = command

        generated = model.generate_inference(prompt, preamble)

        print("LMP:\n")
        print(generated)

if __name__ == '__main__':
    robotest()