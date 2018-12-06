import face_recognition
import cv2
import dlib
import rospy
from std_msgs.msg import String
import time

running = True
class FaceID():

    known_face_encodings=[]
    known_face_names=[]
    counter = 60

    def __init__(self):
        rospy.init_node("faceRec")
        self.running = True
        self.pub = rospy.Publisher("/faceID",String,queue_size=10)
        self.currentTime = time.time()

    def detection(self,path):       #retun the number of face detected (for img), training img must only have one face
        detector = dlib.get_frontal_face_detector()

        print("Processing file: {}".format(path))
        img = dlib.load_rgb_image(path)
        dets = detector(img, 1)
        print("Number of faces detected: {}".format(len(dets)))
        return len(dets)
        # for i, d in enumerate(dets):      #coordinate of the face
        #     print("Detection {}: Left: {} Top: {} Right: {} Bottom: {}".format(
        #         i, d.left(), d.top(), d.right(), d.bottom()))

    def updateLib(self, imgPath, name):       #update face library
        detectedFace = self.detection(imgPath)
        if detectedFace == 0:
            print "Cannot detect a face in the image"
            return False
        elif detectedFace == 1:
            print "Updating library"
            update_img = face_recognition.load_image_file(imgPath)
            update_face_encoding = face_recognition.face_encodings(update_img)[0]
            self.known_face_encodings.append(update_face_encoding)
            self.known_face_names.append(name)
            return True
        elif detectedFace > 1:
            print "Please choose an image with only one face"
            return False
        else:
            print "This shouldn't happen"
            return False

    def start(self,i):
        # Get a reference to webcam #0 (the default one)
        print "FaceID started"
        video_capture = cv2.VideoCapture(i)

        # Load a sample picture and learn how to recognize it.
        # self.updateLib("faceLib/Aaron Yan.jpg","Aaron Yan")
        # self.updateLib("faceLib/Barack Obama.jpg","Barack Obama")
        self.updateLib("faceLib/Dommm.jpg","Dom")
        self.updateLib("faceLib/Hibboman.jpg","Alex")
        self.updateLib("faceLib/Sharkie1.jpg","Sharkie")
        self.updateLib("faceLib/Sharkie2.jpg","Sharkie")
	self.updateLib("faceLib/Siv.jpg", "Siv")

        # Initialize some variables
        face_locations = []
        face_encodings = []
        face_names = []
        # process_this_frame = True
        flag = True
        global running
        while running:
            # Grab a single frame of video
            ret, frame = video_capture.read()
            # print "Getting video: %r" % ret
            if flag:
                # Resize frame of video to 1/4 size for faster face recognition processing
                small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

                # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
                rgb_small_frame = small_frame[:, :, ::-1]

                # Only process every other frame of video to save time
                # if process_this_frame:
                if (time.time() - self.currentTime) >= 1:
                # if self.counter == 0:
                    # self.counter = 60
                    # Find all the faces and face encodings in the current frame of video
                    face_locations = face_recognition.face_locations(rgb_small_frame)
                    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

                    face_names = []
                    for face_encoding in face_encodings:
                        # See if the face is a match for the known face(s)
                        matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding, 0.52)
                    
                        # If a match was found in known_face_encodings, just use the first one.
                        if True in matches:
                            first_match_index = matches.index(True)
                            name = self.known_face_names[first_match_index]
                        else:
                            name = "Unknown"

                        if (time.time() - self.currentTime) >= 1:
                            print name
                            self.pub.publish(name)
                        self.currentTime = time.time()

                        face_names.append(name)

                # process_this_frame = not process_this_frame
                # self.counter = self.counter -1


                # Display the results
                for (top, right, bottom, left), name in zip(face_locations, face_names):
                    if matches:
                        # Scale back up face locations since the frame we detected in was scaled to 1/4 size
                        top *= 4
                        right *= 4
                        bottom *= 4
                        left *= 4
                        # Draw a box around the face
                        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
                        # Draw a label with a name below the face
                        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                        font = cv2.FONT_HERSHEY_DUPLEX
                        cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

            # Display the resulting image
            cv2.imshow('Video', frame)

            # Hit 'q' on the keyboard to quit!            
            if cv2.waitKey(1) == ord('q'):
                print "stopping"
                running = False

        # Release handle to the webcam
        video_capture.release()
        cv2.destroyAllWindows()

    def end(self):
        print "stopping"
        global running
        running = False

if __name__ == '__main__':
    print "starting"
    FaceID().start(1)    # 0 = labtop camera, 1 = kinect camera
