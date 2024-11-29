# LMPVC CodeGen Module

This package contains hierarchical code generation system we use, as well as different implementations for the underlying LLM. Persistent caching support is also included.

**Start the service**
```
ros2 run lmpvc_codegen service
```
## About the different local implementations

**BitsAndBytes**

This is the simplest and most compatible local implementation. Comparatively slow and requires a CUDA enabled GPU with ~11GB of VRAM. Flash Attention 2 could be enabled on 30-series or newer, but on supported hardware we recommend using the other two options instead.

**GPTQ**

Potentially fastest but less compatible, requires a 30-series Nvidia GPU or newer and ~11GB of VRAM.

**GGUF**

Slightly slower than GPTQ but more agressively quantized models are available. Context size can be manually adjusted to further save memory. Requires a 30-series Nvidia GPU or newer and ~10GB of VRAM.

Requirements can be relaxed further by choosing sub-4bit quants, disabling Flash Attention 2 and offloading parts of the model to CPU, at a significant latency cost. These tweaks must be made manually inside *starcoder2_gguf.py*.


## Configuration
```
"model": "gguf",
"inference_api": {
    "hf_url": "bigcode/starcoder2-15b",
    "hf_token": "your_api_token_here"
},
"bitsandbytes":{
    "hf_url": "bigcode/starcoder2-15b",
    "quantization": "4bit"
},
"gptq":{
    "hf_url":"TechxGenus/starcoder2-15b-GPTQ"
},
"gguf":{
    "hf_url": "mradermacher/dolphincoder-starcoder2-15b-i1-GGUF",
    "filename": "dolphincoder-starcoder2-15b.i1-IQ4_XS.gguf",
    "offline": false,
    "context_size": 2048
},
"cache":{
    "enabled": true,
    "filename": null
}
```

**Generic options:**

*model:* Select "inference_api" to run inference on the cloud, or one of the three other options to use local hardware

**Inference API options:**

*hf_url*: Hugging Face id of a model with support for Inference API

*hf_token:* To use Inference API, aquire a HF API token and place it here

**BitsAndBytes options:**

*hf_url*: Hugging Face id of a model supported through HF Transformers

*quantization*: Quantization level, either "8bit" or "4bit"

**GPTQ options:**

*hf_url*: Hugging Face id of a QPTQ based pre-quantized model

**GGUF options:**

*hf_url*: Hugging Face id of repository containing GGUF quant files

*filename:* Name of the file (inside the repository) you want to use

*offline:* Set this **after** a model is downloaded to use it without an internet connection, this is a quirk of the underlying implementation

*context_size*: Size of the input in tokens. Smaller context size saves vram, increase if you get an error message about it.

**Cache options:**

*enabled:* Choose whether to cache generated results for future use

*filename:* If provided, cached results are kept on disk for persistent operation

## Potentially interesting model configurations

We normally use either vanilla Starcoder2-15b or the insturct tuned Dolphincoder version. There seems to be relatively little difference for our use case, but Dolphincoder might have some advantage in terms of natural language understanding. It's also sometimes slightly more verbose when responding through TTS, but we haven't done any large scale comparisons.

Below are two weighted GGUF versions we like to use for optimal VRAM usage without obvious performance pitfalls.

```
"gguf":{
    "hf_url": "mradermacher/starcoder2-15b-i1-GGUF",
    "filename": "starcoder2-15b.i1-IQ4_XS.gguf",
    "offline": false,
    "context_size": 2048
}

"gguf":{
    "hf_url": "mradermacher/dolphincoder-starcoder2-15b-i1-GGUF",
    "filename": "dolphincoder-starcoder2-15b.i1-IQ4_XS.gguf",
    "offline": false,
    "context_size": 2048
}
```

We also present two alternative models we've tested to a limited degree. CodeLlama-34b frequently produces slightly odd TTS output, possibly due to the heavy (2 bit!) quantization, but otherwise works well enough to get some idea of how that architecture might perform. If you have more hardware resources than us, you might want to try CodeLlama-70b, we suspect it could provide benefits to NLU.

```
"gguf":{
    "hf_url": "mradermacher/Phind-CodeLlama-34B-Python-v1-i1-GGUF",
    "filename": "Phind-CodeLlama-34B-Python-v1.i1-IQ2_XXS.gguf",
    "offline": false,
    "context_size": 2048
}

```

Codestral on the other hand was somewhat dissapointing, but we still want to highlight it as an option for interested parties to test. It works for simple commands, but for some reason struggled to understand multi stage instructions (do this and then do that). In the same vein, Codestral seemed reluctant to use undefined functions in the way that's required for hierarchical code generation to take place.

```

"gguf":{
    "hf_url": "mradermacher/Codestral-22B-v0.1-abliterated-v3-i1-GGUF",
    "filename": "Codestral-22B-v0.1-abliterated-v3.i1-IQ3_XXS.gguf",
    "offline": false,
    "context_size": 2048
}
```

UPDATE: We highly recommend testing with various Qwen2.5 variants! Even Qwen2.5-Coder-7B is very competitive with StarCoder2!

```
"gguf":{
    "hf_url": "mradermacher/Qwen2.5-Coder-7B-i1-GGUF",
    "filename": "Qwen2.5-Coder-7B.i1-Q6_K.gguf",
    "offline": false,
    "context_size": 2048
}

"gguf":{
    "hf_url": "mradermacher/Qwen2.5-14B-i1-GGUF",
    "filename": "Qwen2.5-14B.i1-Q4_K_S.gguf",
    "offline": false,
    "context_size": 2048
}

"gguf":{
    "hf_url": "mradermacher/Qwen2.5-32B-i1-GGUF",
    "filename": "Qwen2.5-32B.i1-IQ2_XXS.gguf",
    "offline": false,
    "context_size": 2048
}
```