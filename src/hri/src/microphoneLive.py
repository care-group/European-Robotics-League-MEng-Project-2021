import sounddevice as sd
from scipy.io.wavfile import write

fs = 44100  # Sample rate
seconds = 3  # Duration of recording

print(sd.query_devices())
input_device = 2 # CUSTOMISE THIS
output_device = 1 #CUSTOMISE THIS

myrecording = sd.rec(int(seconds * fs), samplerate=fs, channels=1,device=input_device)
sd.wait()  # Wait until recording is finished

print("starting playback")
sd.play(myrecording, fs,device=output_device)
sd.wait()
print("done")