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

class Spacy_Test:

    def __init__(self):
        rospy.init_node('Spacy_Test')

    def get_text(self):
        text = "go to the kitchen and get a can of coke for Doctor Kimble"
        return text

    def subscribe_cloud(self):
        cloud_subscriber = rospy.Subscriber('/hri/cloud_input', String,self.cloud_callback)
    
    def cloud_callback(self, msg):
        #parse text and execute main code
        self.text = msg.data
        self.string_to_obj(self.check_action())
        self.string_to_obj(self.return_objects())

    def pub_output(self,msg,args):
        output_pub = rospy.Publisher('/hri/cloud_output', String,self.cloud_callback, queue_size=1)
        output_pub.publish(msg) #Publish what the component sees for debugging (as there is a delay due to system performance)


    def string_to_obj(self, string):
        return json.dumps(string)
    
    def obj_to_string(self, obj):
        return json.loads(obj)

    def return_verbs(self):

        nlp = spacy.load("en_core_web_sm")

        print(spacy.__version__)
        doc = nlp(self.text)

        matcher = Matcher(nlp.vocab)
    # Create a pattern matching two tokens: "locate" and a proper noun. This means that the robot knows that it has to complete the 'locate' task and which person to complete the task on
        pattern1 = [{"POS": "VERB"}]
        pattern2 = [{"LEMMA": "be"}]
        pattern3 = [{"LEMMA": "get"}]
        allVerbs = ""
    # Add the pattern to the matcher
        matcher.add("VERBS",[pattern1, pattern2, pattern3])


        for match_id, start, end in matcher(doc):
            allVerbs += (doc[start:end].text)
        return allVerbs

    def get_verbs(self):

        nlp = spacy.load("en_core_web_sm")

        doc = nlp(self.text)

        matcher = Matcher(nlp.vocab)

    # Create a pattern matching two tokens: "locate" and a proper noun. This means that the robot knows that it has to complete the 'locate' task and which person to complete the task on
        pattern1 = [{"POS": "VERB"}]
        pattern2 = [{"LEMMA": "be"}]
        pattern3 = [{"LEMMA": "get"}]

    # Add the pattern to the matcher
        matcher.add("VERBS", None, pattern1, pattern2, pattern3)


        for match_id, start, end in matcher(doc):
            print(doc[start:end].text)
    #print([doc[start:end].text for match_id, start, end in matches])

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
        
            print(roomSpan.text)


    def return_objects(self):

        nlp = spacy.load("en_core_web_sm")

        doc = nlp(self.text)


        objects = ["can of coke", "coke can", "wallet", "purse", "bottle of water"]
        object_patterns = list(nlp.pipe(objects))
        objectMatcher = PhraseMatcher(nlp.vocab)
        objectMatcher.add("OBJECTS", [*object_patterns])


        for match_id, start, end in objectMatcher(doc):

            objectSpan = Span(doc, start, end, label="OBJECTS")

            print(objectSpan.text)
        


    def return_people(self):

        nlp = spacy.load("en_core_web_sm")

        doc = nlp(self.text)


        people = ["Doctor Kimble", "Postman", "Deli Man", "Plumber"]
        people_patterns = list(nlp.pipe(people))
        peopleMatcher = PhraseMatcher(nlp.vocab)
        peopleMatcher.add("PEOPLE", [*people_patterns])

        for match_id, start, end in peopleMatcher(doc):

            peopleSpan = Span(doc, start, end, label="PEOPLE")

            print(peopleSpan.text)


    def check_action(self):
        if "get" in self.return_verbs():
            print("fetch")
            return "fetch"

spacy_test = Spacy_Test()
spacy_test.subscribe_cloud()
rospy.spin()