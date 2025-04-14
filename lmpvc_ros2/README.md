# LMPVC ROS2 Resources

Language Model Program Voice Control (LMPVC) is a voice control framework for robotics, powered by Large Language Models (LLMs).
This ROS2 workspace contains all it's core functionality.

## Instructions

Usage after setting up with one of the procedures further down:

**Direct all local models to the workspace folder:**

```
source hf_setup.bash
```

**Download voices for Piper TTS if using it:**

These go to `models/voices/`, check the PiperTTS GitHub to download different ones: https://github.com/rhasspy/piper

```
./util_scripts/download_piper_voices.bash
```

**Build and source the workspace:**

```
colcon build
```

```
source install/setup.bash
```

**Start the modules required for the system to run:**

```
ros2 run lmpvc_codegen codegen
ros2 run lmpvc_controller controller
ros2 run lmpvc_talker talker
```

**Depending on which functionality you plan to use, enable some or all of the other modules:**

```
ros2 run lmpvc_listener listener
ros2 run lmpvc_detector detector
```

The provided Detector node attempts to contact a corresponding Detector node in the lmpvc_ros1 workspace. Users will most likely want to substitute their own implementation directly in ROS2.

Finally, launch the core application:

```
ros2 run lmpvc_core core
```

**All modules also have corresponding test client in lmpvc_core, e.g.**

```
ros2 run lmpvc_core codegen_test
```
These can run without the core application.

## Configuration

Each module can be further configure in the module_config.json files found in each individual ROS2 package under `./src/module_name/config/`. You should at least know what's in core_config.json. Usage of the config files is explained in a separate README.md in each individual directory. If you're not using symlink build, the workspace will always have to be rebuilt for configuration changes to take effect.

## Docker setup

Helper scripts in *./docker_scripts/* can be used to build, run and stop the container, as well as run bash terminals while it is running. Building and running use different scripts depending on wheter you're interested in the *cpu* or *cuda* version of the container. It is recommended to use *cuda* for full functionality, which currently assumes at least a 3000-series NVIDIA GPU and a Docker installation configured to use NVIDIA Container Toolkit:

https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

Additionally, to build the *cuda* image, add *"default-runtime": "nvidia"* to */etc/docker/daemon.json* (assuming default location):

```
{
    "runtimes": {
        "nvidia": {
            "path": "/usr/bin/nvidia-container-runtime",
            "runtimeArgs": []
         } 
    },
    "default-runtime": "nvidia" 
}
```

Then restart the docker service:

```
sudo systemctl restart docker
```

Now, it should be possible to use the provided script to start the building process.

Co-workers: Contact me for pre-built images!

## Requirements and venv setup

This section is most likely not a complete set of requirements for all systems, but it tries to list the main ones. There's also some additional advice for if you want to use a virtual environment, which is unfortunately a little convoluted.

### ROS2

If you don't have ROS2, install it according to the documentation of whichever version you plan to use. This package was tested and developed using Foxy and Humble.

Humble installation:

https://docs.ros.org/en/humble/Installation.html

### Setup

These instructions assume you're using a venv environment, just disregard any venv-related instructions if installing directly.

First, initialize a new venv.

```
python3 -m venv --system-site-packages venv
```
If you don't use --system-site-packages, getting ROS2 working will be require more work, which is not covered here. In general, ROS2 is badly designed for VENV situations, so you need to add

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

anytime after sourcing the venv to always call colcon build with the right arguments. When building, there may be a ton of random warnings, that seems to be just be how Colcon likes it.

### Requirements

**Everything ROS2 needs**

Install OUTSIDE the venv for a less confusing experience

```
apt-get update && apt-get install -y \
pulseaudio \
ros-humble-moveit \
alsa-utils \
pulseaudio-utils \
espeak \
ffmpeg \
libespeak1 \
portaudio19-dev \
libasound2-plugins \
```

then use *--system-site-packages* to build your venv. 

**Generic Python libraries**
```
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir --default-timeout=1000 \
    # Older NumPy required for some PyTorch components
    numpy==1.24.1  \
    pyaudio \
    pyttsx3 \
    sounddevice \
    piper-tts \
    astunparse
```

**Astunparse**

Required for the hierarchical code generation

```
pip install astunparse
```

**PyTorch**

For the right configuration for your hardware, use the official website to generate a pip command:

https://pytorch.org/

```
pip install --no-cache-dir \
torch \
torchvision \
torchaudio \
--index-url https://download.pytorch.org/whl/cu124
```

**Transformers**

Install after PyTorch. StarCoder2 recommends installing from source:

```
pip install --no-cache-dir git+https://github.com/huggingface/transformers.git \
accelerate \
bitsandbytes
```

**GGUF**

For CUDA:

```
pip install flash-attn --no-build-isolation
```

```
CMAKE_ARGS="-DGGML_CUDA=on" pip install llama-cpp-python
```

Other options:

https://github.com/abetlen/llama-cpp-python


**QPTQ**

Optional, GGUF recommended

```
pip install flash-attn --no-build-isolation
```

```
pip install optimum[onnxruntime-gpu] \
git+https://github.com/PanQiWei/AutoGPTQ
```
