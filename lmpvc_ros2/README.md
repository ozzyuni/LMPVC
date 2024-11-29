# LMPVC ROS2 Resources

Language Model Program Voice Control (LMPVC) is a voice control framework for robotics, powered by Large Language Models (LLMs).
This ROS2 workspace contains all it's core functionality.

## Instructions

**Shortcut to set up the environment on the Robolab CV PC:**
```
ossi_ros2
```
**Start the modules required for the system to run:**
```
ros2 run lmpvc_codegen service
ros2 run lmpvc_controller service
ros2 run lmpvc_talker service
```
**Depending on which functionality you plan to use, enable some or all of the other modules:**
```
ros2 run lmpvc_listener server
ros2 run lmpvc_detector service
```
For the controller and detector, remember to launch the corresponing ROS1 adapter nodes in the accompanying ROS1 workspace!

Finally, launch the core application:
```
ros2 run lmpvc_core voice_control
```

**All modules also have corresponding test function in lmpvc_core, e.g.**

```
ros2 run lmpvc_core codegen_test
```
These can run without the core application.

## Configuration

Each module can be further configure in the module_config.json files found in each individual ROS2 package under ./src/module_name/module_name. You should at least know what's in core_config.json. Usage of the config files is explained in a separate README.md in each individual directory.

## Requirements and venv setup

This section is most likely not a complete set of requirements, but it tries to list the main ones. There's also some additional advice for if you want to use a virtual environment, which is unfortunately a little convoluted.

If you're doing this in Robolab, I recommend you check out *venv.txt* for slightly more detailed information.

### ROS2

If you don't have ROS2, install it according to the documentation of whichever version you plan to use. This package was tested and developed using Foxy and Humble.

Humble installation:

https://docs.ros.org/en/humble/Installation.html

### Venv setup

If you don't use --system-site-packages, you have to install more packages manually to make
ROS2 able to build the ws, and/or work at all.

```
python3 -m venv --system-site-packages venv
```
In general, ROS2 is badly designed for VENV situations, so you need to add

```
export PYTHONPATH=$PYTHONPATH:"$VIRTUAL_ENV/lib/python3.8/site-packages"
```

to *venv/bin/activate*, or do something equivalent elsewhere. We also recommend an alias for easier building:

```
alias lmpvc_build='colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python"'
```

With this in your /bin/activate, you can simply use the command

```
lmpvc_build
```

or

```
lmpvc_build --packages-select *package1* *package2* ...
```

anytime after sourcing the venv to always call colcon build with the right arguments. When building, expect a ton of random warnings, that seems to just be how Colcon likes it.

### Python requirements

**Everything ROS2 needs**

You should hopefully already have this, unless you didn't use *--system-site-packages* to build your venv.

**Audio**

Requirede for TTS.

```
sudo apt install espeak ffmpeg
pip install pyaudio
pip install pyttsx3 
```

**Astunparse**

Required for the hierarchical code generation

```
pip install astunparse
```

**PyTorch**

For the right configuration for your hardware, use the official website to generate a pip command:

https://pytorch.org/

**Transformers**

Install after PyTorch. StarCoder2 recommends installing from source:

```
pip install --no-cache-dir git+https://github.com/huggingface/transformers.git
```

**Accelerate**

```
pip install --no-cache-dir accelerate
```

**BitsAndBytes**

You may only install the quantization method you need, see *src/lmpvc_codegen/lmpvc_codegen/*

```
pip install --no-cache-dir bitsandbytes
```

**QPTQ**

```
pip install flash-attn --no-build-isolation
pip install optimum[onnxruntime-gpu]
pip install git+https://github.com/PanQiWei/AutoGPTQ
```

**GGUF**

For CUDA:

```
pip install flash-attn --no-build-isolation
CMAKE_ARGS="-DGGML_CUDA=on" pip install llama-cpp-python
```

Other options:

https://github.com/abetlen/llama-cpp-python