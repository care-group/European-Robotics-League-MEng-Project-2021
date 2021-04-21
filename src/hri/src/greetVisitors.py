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

class Greet_Visitors:

    def __init__(self):
        rospy.init_node('Greet_Visitors')

    #def get_text(self):
     #   text = "go to the kitchen and get a can of coke for Doctor Kimble"
      #  return text

    def subscribe_greet(self):
        print("subscriber is called")
        greet_subscriber = rospy.Subscriber('/hri/greet_input', String,self.greet_callback)
    
    def greet_callback(self, msg):
        #parse text and execute main code

        print("first has been reached")
        self.text = msg.data

        print(self.text)

        self.string_to_tts("Hi, welcome to the home of Grannie Annie, could you please state your name and how can I help you today")

        #dictMsg={}
        #if(self.recognised_visitor() is not None):
        #    dictMsg["person"]=self.recognised_visitor().text
        #if(self.recognised_visitor().text is "plumber"):
        #    dictMsg["room"]=self.ask_plumber().text
        
        #dictWrapper=dictMsg
        #jsonStr = json.dumps(dictWrapper)
        #print(jsonStr)
        #output_pub = rospy.Publisher('/hri/greet_output', String, queue_size=1,latch=True)
        #output_pub.publish(jsonStr) #Publish what the component sees for debugging (as there is a delay due to system performance)

    def string_to_tts(self, string):
        tts_pub = rospy.Publisher('/hri/tts_input', String, queue_size=1,latch=True)
        tts_pub.publish(string) #Publish what the component sees for debugging (as there is a delay due to system performance)

    def string_to_obj(self, string):
        return json.dumps(string)
    
    def obj_to_string(self, obj):
        return json.loads(obj)

    def recognised_visitor(self):
        if("doctor kimble" in self.return_people()):
            return "Dr Kimble"
        if("postman" in self.return_people()):
            return "postman"
        if("deli man" in self.return_people()):
            return "deli man"
        if("plumber" in self.return_people()):
            return "plumber"

    def ask_plumber(self):
        if("kitchen" in self.return_rooms()):
            return "kitchen"
        if("bathroom" in self.return_rooms()):
            return "bathroom"

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


        people = ["doctor kimble", "postman", "deli man", "plumber"]
        people_patterns = list(nlp.pipe(people))
        peopleMatcher = PhraseMatcher(nlp.vocab)
        peopleMatcher.add("PEOPLE", [*people_patterns])

        for match_id, start, end in peopleMatcher(doc):

            peopleSpan = Span(doc, start, end, label="PEOPLE")

            return peopleSpan


greet_visitors = Greet_Visitors()
greet_visitors.subscribe_greet()
rospy.spin()