import spacy
import json
from spacy.matcher import PhraseMatcher
from spacy.matcher import Matcher
from spacy.tokens import Span
from spacy.lang.en import English



nlp = spacy.load("en_core_web_sm")


#people = ["Doctor Kimble", "Postman", "Deli Man", "Plumber"]
#people_patterns = list(nlp.pipe(people))
#matcher.add("PEOPLE", None, *people_patterns)

#objects = ["can of coke", "coke can", "wallet", "purse", "bottle of water"]
#object_patterns = list(nlp.pipe(objects))
#matcher.add("OBJECTS", None, *object_patterns)


TEXT = "go to the kitchen and get a can of coke for Doctor Kimble"

doc = nlp(TEXT)

matcher = Matcher(nlp.vocab)

def return_verbs():

# Create a pattern matching two tokens: "locate" and a proper noun. This means that the robot knows that it has to complete the 'locate' task and which person to complete the task on
    pattern1 = [{"POS": "VERB"}]
    pattern2 = [{"LEMMA": "be"}]
    pattern3 = [{"LEMMA": "get"}]
    allVerbs = ""
# Add the pattern to the matcher
    matcher.add("VERBS", None, pattern1, pattern2, pattern3)

    matches = matcher(doc)

    for match_id, start, end in matcher(doc):
        allVerbs += (doc[start:end].text)
    return allVerbs

def get_verbs():

# Create a pattern matching two tokens: "locate" and a proper noun. This means that the robot knows that it has to complete the 'locate' task and which person to complete the task on
    pattern1 = [{"POS": "VERB"}]
    pattern2 = [{"LEMMA": "be"}]
    pattern3 = [{"LEMMA": "get"}]

# Add the pattern to the matcher
    matcher.add("VERBS", None, pattern1, pattern2, pattern3)

    matches = matcher(doc)

    for match_id, start, end in matcher(doc):
        print(doc[start:end].text)
#print([doc[start:end].text for match_id, start, end in matches])

def return_rooms():

    rooms = ["kitchen", "bedroom", "bathroom", "hallway", "living room"]
    room_patterns = list(nlp.pipe(rooms))
    roomMatcher = PhraseMatcher(nlp.vocab)
    roomMatcher.add("ROOM", None, *room_patterns)

    for match_id, start, end in roomMatcher(doc):
        # Create a Span with the label for "GPE"
        roomSpan = Span(doc, start, end, label="ROOM")
        #peopleSpan = Span(doc, start, end, label="PEOPLE")
        #objectSpan = Span(doc, start, end, label="OBJECTS")
        # Overwrite the doc.ents and add the span
       # doc.ents = list(doc.ents) + [roomSpan] + [peopleSpan] + [objectSpan]

        roomSpan_root_head = roomSpan.root.head
    
        # Print the text of the span root's head token and the span text
        print(roomSpan.text)
        # Print the text of the span root's head token and the span text
        #print(roomSpan.text)

def return_objects():

    objects = ["can of coke", "coke can", "wallet", "purse", "bottle of water"]
    object_patterns = list(nlp.pipe(objects))
    objectMatcher = PhraseMatcher(nlp.vocab)
    objectMatcher.add("OBJECTS", None, *object_patterns)


    for match_id, start, end in objectMatcher(doc):
        # Create a Span with the label for "GPE"
        objectSpan = Span(doc, start, end, label="OBJECTS")
        #peopleSpan = Span(doc, start, end, label="PEOPLE")
        #objectSpan = Span(doc, start, end, label="OBJECTS")
        # Overwrite the doc.ents and add the span
       # doc.ents = list(doc.ents) + [roomSpan] + [peopleSpan] + [objectSpan]

    
        # Print the text of the span root's head token and the span text
        print(objectSpan.text)
        # Print the text of the span root's head token and the span text
       # print(objectSpan.text)

def return_people():

    people = ["Doctor Kimble", "Postman", "Deli Man", "Plumber"]
    people_patterns = list(nlp.pipe(people))
    peopleMatcher = PhraseMatcher(nlp.vocab)
    peopleMatcher.add("PEOPLE", None, *people_patterns)

    for match_id, start, end in peopleMatcher(doc):
        # Create a Span with the label for "GPE"
        peopleSpan = Span(doc, start, end, label="PEOPLE")
        #peopleSpan = Span(doc, start, end, label="PEOPLE")
        #objectSpan = Span(doc, start, end, label="OBJECTS")
        # Overwrite the doc.ents and add the span
       # doc.ents = list(doc.ents) + [roomSpan] + [peopleSpan] + [objectSpan]

    
        # Print the text of the span root's head token and the span text
        print(peopleSpan.text)
        # Print the text of the span root's head token and the span text
        #print(peopleSpan.text)

def check_action():
    if "get" in return_verbs():
        print("fetch")
        

check_action()
return_objects()