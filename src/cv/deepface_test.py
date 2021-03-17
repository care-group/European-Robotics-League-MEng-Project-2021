from deepface import DeepFace
import pandas as pd
import pickle

def check_if_faces_match(img1,img2):
    result  = DeepFace.verify(img1,img2)
    print("Input Images:"+"\n\t"+img1 + "\n\t" +img2+"\nResult:")
    if result.get("verified") is True:
        print("\tSame faces")
    else:
        print("\tDifferent faces")


check_if_faces_match("images/faces/bezos_1.jpg", "images/faces/bezos_2.jpg")
check_if_faces_match("images/faces/bezos_1.jpg", "images/faces/bezos_3.jpg")
check_if_faces_match("images/faces/bezos_1.jpg", "images/faces/samuel_1.jpg")
