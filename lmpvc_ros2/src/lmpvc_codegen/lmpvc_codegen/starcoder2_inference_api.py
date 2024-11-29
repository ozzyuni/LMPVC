#!/usr/bin/env python

# This module implements StarCoder2 with Hugging Face Inference API

import json
import requests
from pathlib import Path

class Model:
    """Inference API implementation of the StarCoder2 code generating LLM"""
     
    def __init__(self):
        config_path = Path(__file__).with_name('codegen_config.json')
        config = {}
        with open(config_path, 'r') as config_file:
            config = json.load(config_file)["inference_api"]

        self.api_url = 'https://api-inference.huggingface.co/models/' + config['hf_url']
        self.headers = {'Authorization': 'Bearer ' + config['hf_token']}

    def query(self, payload):
        inference = ''
        try:
            response = requests.post(self.api_url, headers=self.headers, json=payload)
            data = response.json()
            inference = data[0]['generated_text']
        except requests.exceptions.RequestException as e:
            print("Request exception:", e)
            inference = ''

        return inference

    def generate_inference(self, prompt, preamble = '', template=('', ''), stop_tokens=[], log=False):
        """Provides a unified interface for generating inference with different backends."""
        if log:
            print("\nCalling Inference API!")

        prompt = template[0] + prompt + template[1]

        inference = self.query({'inputs': preamble + prompt,
                           'parameters': {'do_sample': False, 'max_new_tokens': 500,'return_full_text': False, 'stop': stop_tokens},
                           'options': {'use_cache': False, 'wait_for_model': True}})

        return inference

def demo():
    model = Model()

    while(True):
        prompt = input('Instruction: ')
        generated = model.generate_inference(prompt, template=('# ', '\n'))
        print("\n" + prompt + generated)

if __name__ == "__main__":
    demo()
