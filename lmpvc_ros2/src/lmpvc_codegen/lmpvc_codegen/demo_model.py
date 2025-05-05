#!/usr/bin/env python

# This module is a placeholder for demo purposes

class Model:
    """Placeholder Model class for demoing the system without a model loaded"""
     
    def __init__(self, msg="[ERROR] Can't generate code, no model loaded"):
        self.msg = msg

    def generate_inference(self, prompt, preamble = '', template=('', ''), stop_tokens=[], log=False):
        """Prints a warning message if code generation is called unexpectedly"""
        print(self.msg)
        return ""

def demo():
    model = Model()

    while(True):
        prompt = input('Instruction: ')
        generated = model.generate_inference(prompt, template=('# ', '\n'))
        print("\n" + prompt + generated)

if __name__ == "__main__":
    demo()
