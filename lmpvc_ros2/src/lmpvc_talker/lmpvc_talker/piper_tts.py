#!/usr/bin/env python
import json
import os
import numpy as np
import sounddevice as sd
from piper.voice import PiperVoice
from pathlib import Path

class Talker:
    def __init__(self):
        talker_config_path = ""
        try:
            from ament_index_python.packages import get_package_share_directory

            talker_config_path = os.path.join(
                get_package_share_directory('lmpvc_talker'),
                'talker_config.json'
            )
        except:
            talker_config_path = Path(__file__).with_name('talker_config.json')

        modelpath = ""
        with open(talker_config_path, 'r') as config_file:
            modelpath = json.load(config_file)['piper']['model']
        
        self.model = PiperVoice.load(modelpath)
    
    def say(self, utterance):
        try:
            stream = sd.OutputStream(samplerate=self.model.config.sample_rate, channels=1, dtype='int16')
            stream.start()

            for audio_bytes in self.model.synthesize_stream_raw(utterance):
                int_data = np.frombuffer(audio_bytes, dtype=np.int16)
                stream.write(int_data)
                
            stream.stop()
            stream.close()
            return True
        
        except Exception as e:
            print(e)
            return False

if __name__ == '__main__':
    voice = Talker()
    
    while True:
        cmd = input("Write something to say (q to quit): ")
        if cmd == 'q':
            break
        voice.say(cmd)