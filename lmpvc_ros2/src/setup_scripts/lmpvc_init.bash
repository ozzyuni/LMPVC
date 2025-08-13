#!/bin/bash
mkdir -p ${PWD}/models/voices
wget https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_GB/northern_english_male/medium/en_GB-northern_english_male-medium.onnx -P ${PWD}/models/voices/
wget https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_GB/northern_english_male/medium/en_GB-northern_english_male-medium.onnx.json -P ${PWD}/models/voices/

wget https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_GB/jenny_dioco/medium/en_GB-jenny_dioco-medium.onnx -P ${PWD}/models/voices/
wget https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_GB/jenny_dioco/medium/en_GB-jenny_dioco-medium.onnx.json -P ${PWD}/models/voices/

. /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

cp ${PWD}/src/setup_scripts/lmpvc_ws.bash ${PWD}/lmpvc_ws.bash