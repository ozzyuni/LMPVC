#!/usr/bin/env python3
import json
import torch
import os
from pathlib import Path
from transformers import AutoTokenizer, AutoModelForCausalLM, BitsAndBytesConfig,StoppingCriteria

class StopSequence(StoppingCriteria):
    """Implements a custom stopping criteria, effectively the same as the stop_tokens
        parameter other implementations can accept directly.
    """
    def __init__(self, target_sequences, prompt, tokenizer):
        super().__init__()
        self.target_sequences = target_sequences
        self.tokenizer = tokenizer
        self.prompt = prompt
    
    def __call__(self, input_ids, scores, **kwargs):
        generated_text = self.tokenizer.decode(input_ids[0])
        generated_text = generated_text.replace(self.prompt, '')

        #print("\nGenerated text:")
        #print(generated_text)

        for target_sequence in self.target_sequences:
            #print("Looking for stop token: ", target_sequence)
            if target_sequence in generated_text:
                return True
        return False
    
    def __len__(self):
        return 1
    
    def __iter__(self):
        yield self

class Model:
    """HuggingFace Transformers implementation of the LLM interface. Supports BitsAnsBytes
        and QPTQ quantizations.
    """
    def __init__(self):
        config_path = ""
        try:
            from ament_index_python.packages import get_package_share_directory

            config_path = os.path.join(
                get_package_share_directory('lmpvc_codegen'),
                'codegen_config.json'
            )
        except:
            config_path = Path(__file__).with_name('codegen_config.json')

        config = {}
        device_map = 'cuda'
        with open(config_path, 'r') as config_file:
            config = json.load(config_file)
            self.mode = config['model']
            config = config[self.model]

        if config.get('quantization') == '4bit':
            self.quantization_config = BitsAndBytesConfig(load_in_4bit=True,
                                                            bnb_4bit_use_double_quant=True,
                                                            bnb_4bit_quant_type='nf4',
                                                            bnb_4bit_compute_dtype=torch.bfloat16,
                                                            )
        elif config.get('quantization') == '8bit':
            device_map = 'auto'
            self.quantization_config = BitsAndBytesConfig(load_in_8bit=True,
                                                            llm_int8_enable_fp32_cpu_offload=True
                                                            )

        if self.mode == 'gptq':
            self.checkpoint = config['hf_url']
            self.tokenizer = AutoTokenizer.from_pretrained(self.checkpoint)
            self.model = AutoModelForCausalLM.from_pretrained(self.checkpoint,
                            device_map = 'cuda',
                            attn_implementation="flash_attention_2")

        else:
            self.checkpoint = config['hf_url']
            self.tokenizer = AutoTokenizer.from_pretrained(self.checkpoint)
            self.model = AutoModelForCausalLM.from_pretrained(self.checkpoint,
                            quantization_config = self.quantization_config, device_map=device_map)
    
    def generate_inference(self, prompt, preamble = '', template=('', ''), stop_tokens=[], log=False):
        """Provides a unified interface for generating inference with different backends."""
        torch.cuda.empty_cache()

        prompt = template[0] + prompt + template[1]
        
        inputs = self.tokenizer.encode(preamble + prompt, return_tensors='pt').to('cuda')

        token_count = torch.count_nonzero(inputs).item()

        if log:
            print("\nCalling local generation!")
            print("Input tokens:", token_count)

        if token_count > 4096:
            print("\nWarning: Number of input tokens >4096, relying on sliding attention window!")

        outputs = self.model.generate(inputs, max_new_tokens=300, do_sample=False,
                                      stopping_criteria=StopSequence(stop_tokens, preamble + prompt, self.tokenizer))
        
        generated_text = self.tokenizer.decode(outputs[0])
        generated_text = generated_text.replace(preamble + prompt, '')

        return generated_text

def main():
    """Simple test."""
    model = Model()
    print("Model ready! Write a prompt, or 'q' to quit.")
    while True:
        prompt = input("\nPrompt:")
        if prompt == 'q':
            break
        generated = model.generate_inference(prompt, template=('# ', '\n'))
        print("\n" + prompt + generated)


if __name__ == '__main__':
    main()

