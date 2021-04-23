#!/usr/bin/python

import json
import numpy as np
import rospy
from std_msgs.msg import String
from google.cloud import texttospeech
import sounddevice as sd
import soundfile as sf
import os 
class TextToSpeech:

    def __init__(self):
       rospy.init_node('TextToSpeech')

    def subscribe_tts(self):
        tts_subscriber = rospy.Subscriber('/hri/tts_input', String,self.tts_callback)
    
    def tts_callback(self, msg):
        self.text = msg.data
        self.convert_to_tts(self.text)

        tts_pub = rospy.Publisher('/hri/tts_output', String, queue_size=1,latch=True)
        tts_pub.publish('done')


    def convert_to_tts(self,textIn):
        client = texttospeech.TextToSpeechClient()

        # Set the text input to be synthesized
        synthesis_input = texttospeech.SynthesisInput(text= textIn)

        # Build the voice request, select the language code ("en-US") and the ssml
        # voice gender ("neutral")
        voice = texttospeech.VoiceSelectionParams(
            language_code="en-US", ssml_gender=texttospeech.SsmlVoiceGender.NEUTRAL
        )

        # Select the type of audio file you want returned
        audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.MP3
        )

        # Perform the text-to-speech request on the text input with the selected
        # voice parameters and audio file type
        response = client.synthesize_speech(
            input=synthesis_input, voice=voice, audio_config=audio_config
        )


        out_path = '/home/developer/workspace/src/hri/src/output.mp3'
        # The response's audio_content is binary.
        with open(out_path, 'wb') as out:
            # Write the response to the output file.
            out.write(response.audio_content)
            print('Audio content written to file "output.mp3"')
            mystring = "mpg123 " + out_path
            print(mystring)
            os.system(mystring)


TextToSpeech = TextToSpeech()
TextToSpeech.subscribe_tts()
rospy.spin()
