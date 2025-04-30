#!/usr/bin/env python
import time
import os
import json
from pathlib import Path
from lmpvc_talker.piper_tts import Talker

class DemoHandler:
  def __init__(self):

    config_path = ""

    try:
      from ament_index_python.packages import get_package_share_directory

      config_path = os.path.join(
          get_package_share_directory('lmpvc_core'),
          'demo_config.json'
      )
    except:
      config_path = Path(__file__).with_name('demo_config.json')
    
    config = {}

    with open(config_path, 'r') as config_file:
        config = json.load(config_file)

    self.cmd_index = 0
    self.cmds = config['commands']
    self.voice = Talker(model_path=config['model_path'])

  def simulate_voice_command(self):
    cmd = self.cmds[self.cmd_index]
    self.voice.say(cmd)

    self.cmd_index += 1
    self.cmd_index = self.cmd_index if self.cmd_index < len(self.cmds) else 0

    return cmd

  def simulate_text_command(self):
    cmd = self.cmds[self.cmd_index]
    time.sleep(1)

    self.cmd_index += 1
    self.cmd_index = self.cmd_index if self.cmd_index < len(self.cmds) else 0

    return cmd
  
def main():
    demo = DemoHandler()
    for i in range(10):
       cmd = demo.simulate_voice_command()
       print("[COMMAND {i}] {cmd}".format(i=i, cmd=cmd))

if __name__ == '__main__':
    main()