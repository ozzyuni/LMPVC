#!/usr/bin/env python3
import json
from pathlib import Path
from llama_cpp import Llama

class Model:
    """Implements LLM access with llama.cpp, making it possible to use quantized models
        in the GGUF format. While it's possible to offload parts of these models to the 
        cpu, this class is configured for gpu only (n_gpu_layers=-1).
    """
    def __init__(self):
        config_path = Path(__file__).with_name('codegen_config.json')
        config = {}
        with open(config_path, 'r') as config_file:
            config = json.load(config_file)['gguf']

        hf_path = config['hf_url'].split('/')
        local_dir = Path(__file__).resolve().parent / 'models' / hf_path[-2] / hf_path[-1] 
        local_dir.mkdir(parents=True, exist_ok=True)
        local_file = local_dir / config['filename']

        if config['offline'] == True:
            # The llama-cpp-python implementation of from_pretrained does not behave
            # as expected without internet access, resulting in an error. To avoid
            # this, in offline mode we initialize the Llama class directly.
            self.model = Llama(model_path=str(local_file),
                                n_ctx=config['context_size'],
                                n_gpu_layers=-1,
                                flash_attn=True)

        else:
            # However, we still want to be able to automatically download models if
            # an internet connection is available. For this, we have to keep using
            # Llama.from_pretrained.
            self.model = Llama.from_pretrained(repo_id=config['hf_url'],
                                        filename=config['filename'],
                                        verbose=False,
                                        local_dir=local_dir,
                                        n_ctx=config['context_size'],
                                        n_gpu_layers=-1,
                                        flash_attn=True)
        
    def generate_inference(self, prompt, preamble = '', template=('', ''), stop_tokens=[], log=False):
        """Provides a unified interface for generating inference with different backends."""
        prompt = template[0] + prompt + template[1]
        
        generated_text = self.model(preamble + prompt,
                                    max_tokens=500,
                                    stop=stop_tokens,
                                    echo=False,
                                    temperature=0.0)['choices'][0]['text']

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

