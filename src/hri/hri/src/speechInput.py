#!/usr/bin/python

import sounddevice as sd
import speech_recognition as sr
import pyaudio

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty

from hri.srv import SpeechToText

r = sr.Recognizer()

def listenToSpeech(msg):
    # initialize the recognizer
    #print(sr.Microphone.list_working_microphones())
    #print("Make sure you select the right device (0,1,2...)")
   

    with sr.Microphone(device_index=2, sample_rate = 48000) as source:
        # read the audio data from the default microphone
        print("Listening to audio..")
        #r.adjust_for_ambient_noise(source)
        audio = r.listen(source,timeout=5)
        text =r.recognize_google(audio,language='en-GB')
        
        #print(text)
        output = String()
        output.data=text
        return{'rawText':output}
        

if __name__ == '__main__':
    rospy.init_node('speechInput')
    rospy.Service('speechInput',SpeechToText,listenToSpeech)
    
    try:
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logerr(e)