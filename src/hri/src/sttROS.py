import sounddevice as sd
import speech_recognition as sr
import pyaudio
import rospy
from std_msgs.msg import String



def talker(text):
    pub = rospy.Publisher('/hri/cloud_output', String, queue_size=10)
    pub.publish(text)
        

if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
        # initialize the recognizer
    r = sr.Recognizer()
    #print(sr.Microphone.list_working_microphones())
    print("Make sure you select the right device (0,1,2...)")
   

    with sr.Microphone(device_index=0, sample_rate = 44100) as source:
        # read the audio data from the default microphone
        print("Listening to audio..")
        r.adjust_for_ambient_noise(source)
        audio = r.listen(source,timeout=5)
        text =r.recognize_google(audio,language='en-GB')
        print(text)
        talker(text)
    rospy.spin()
   
