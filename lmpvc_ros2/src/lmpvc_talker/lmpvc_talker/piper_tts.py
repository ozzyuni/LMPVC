#!/usr/bin/env python
import numpy as np
import sounddevice as sd
from piper.voice import PiperVoice
from pathlib import Path

class Talker:
    def __init__(self):
        modelpath = str(Path(__file__).parent / "voices/en_GB-northern_english_male-medium.onnx")
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