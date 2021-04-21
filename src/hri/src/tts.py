import spacy
from google.cloud import texttospeech
from google.cloud import texttospeech_v1

class TTS_Test:

    def __init__(self):
        rospy.init_node('TTS_Test')

    def convert_to_tts(self,msg):
        self.tts = gTTS('hello')
        self.tts.save('hello.mp3')

    def subscribe_tts(self):
        tts_subscriber = rospy.Subscriber('/hri/tts_input', String,self.subscribe_callback)

    def subscribe_callback(self, msg):
        #parse text and execute main code
        self.text = msg.data

        self.convert_to_tts().text


        dictWrapper=dictMsg
        jsonStr = json.dumps(dictWrapper)
        print(jsonStr)
        output_pub = rospy.Publisher('/hri/tts_output', String, queue_size=1,latch=True)
        output_pub.publish(jsonStr) #Publish what the component sees for debugging (as there is a delay due to system performance)

tts_test = TTS_Test()
tts_test.subscribe_tts()
rospy.spin()

