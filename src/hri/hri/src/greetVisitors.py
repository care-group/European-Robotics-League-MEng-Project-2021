#!/usr/bin/python
import spacy
import json
import numpy as np
import rospy
from std_msgs.msg import String
from spacy.matcher import PhraseMatcher
from spacy.matcher import Matcher
from spacy.tokens import Span
from spacy.lang.en import English
from hri.srv import SpeechToText

class Greet_Visitors:

    def __init__(self):
        rospy.init_node('Greet_Visitors')
        self.tts_pub = rospy.Publisher('/hri/tts_input', String, queue_size=1,latch=True)
        self.greet_pub = rospy.Publisher('/hri/greet_output', String, queue_size=10)
        self.room_pub = rospy.Publisher('/hri/location_output', String, queue_size=10)

    #def get_text(self):
     #   text = "go to the kitchen and get a can of coke for Doctor Kimble"
      #  return text

    def subscribe_jason(self):
        #print("subscriber is called")
        jason_subscriber = rospy.Subscriber('/hri/greet_input', String,self.greet_callback)
        location_subscriber = rospy.Subscriber('/hri/location_input', String,self.location_callback)

    def greet_callback(self, msg):
        #parse text and execute main code

        rospy.wait_for_service('speechInput')
        getSpeechInput =rospy.ServiceProxy('speechInput',SpeechToText)

        resp = getSpeechInput()
        print("Received input from service:")
        print(resp.rawText.data)
        self.text = resp.rawText.data

        
        visitor=self.recognised_visitor()
        # if(self.recognised_visitor() == "plumber"):
        #     dictMsg["room"]=self.ask_plumber()

        self.greet_pub.publish(visitor)
        print("published in theory")

    def location_callback(self, msg):
        #parse text and execute main code

        rospy.wait_for_service('speechInput')
        getSpeechInput =rospy.ServiceProxy('speechInput',SpeechToText)

        resp = getSpeechInput()
        print("Received input from service:")
        print(resp.rawText.data)
        self.text = resp.rawText.data

        
        room=self.recognised_room()


         
        print(room)
        self.room_pub.publish(room) #Publish what the component sees for debugging (as there is a delay due to system performance)

    def string_to_tts(self, string):
        self.tts_pub.publish(string) #Publish what the component sees for debugging (as there is a delay due to system performance)

    def string_to_obj(self, string):
        return json.dumps(string)
    
    def obj_to_string(self, obj):
        return json.loads(obj)

    def recognised_visitor(self): 
        if(self.return_people() is not None):
            return self.return_people().text
        else: 
            return "unrecognised"
            
    def recognised_room(self): 
        if(self.return_rooms() is not None):
            return self.return_rooms().text
        else: 
            return "unrecognised"

    def ask_plumber(self):
        if(self.return_rooms() is not None):
            return self.return_rooms().text

    def return_rooms(self):

        nlp = spacy.load("en_core_web_sm")

        doc = nlp(self.text)


        rooms = ["kitchen", "bedroom", "bathroom", "hallway", "living room"]
        room_patterns = list(nlp.pipe(rooms))
        roomMatcher = PhraseMatcher(nlp.vocab)
        roomMatcher.add("ROOM", [*room_patterns])

        for match_id, start, end in roomMatcher(doc):
            # Create a Span with the label for "GPE"
            roomSpan = Span(doc, start, end, label="ROOM")
            
            return roomSpan
    
    def return_people(self):

        nlp = spacy.load("en_core_web_sm")

        doc = nlp(self.text)


        people = ["doctor", "doctor kimble", "postman", "deli man", "plumber"]
        people_patterns = list(nlp.pipe(people))
        peopleMatcher = PhraseMatcher(nlp.vocab)
        peopleMatcher.add("PEOPLE", [*people_patterns])

        for match_id, start, end in peopleMatcher(doc):

            peopleSpan = Span(doc, start, end, label="PEOPLE")

            return peopleSpan


greet_visitors = Greet_Visitors()
greet_visitors.subscribe_jason()
rospy.spin()