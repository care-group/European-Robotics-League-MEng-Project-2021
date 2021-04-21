import sounddevice as sd
import speech_recognition as sr

# initialize the recognizer
r = sr.Recognizer()

print("Make sure you select the right device (0,1,2...)")
print(sr.Microphone.list_microphone_names())

with sr.Microphone(device_index=2,sample_rate = 48000) as source:
    # read the audio data from the default microphone
    print("Listening to audio..")
    r.adjust_for_ambient_noise(source)
    audio = r.listen(source,timeout=2)
    text =r.recognize_google(audio,language='en-GB',show_all=True)
    print(text)
