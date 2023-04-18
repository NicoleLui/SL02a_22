#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge, CvBridgeError

class ImageHandler:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_identified_frame = np.zeros((480,640,3),dtype=np.uint8)
        self.image_raw_frame = []
        self.image_raw_hold = np.zeros((480,640,3),dtype=np.uint8)
        #self.image_raw = np.zeros((640,480,3),dtype=np.uint8)



        self.thres = 0.55 # Threshold to detect object
        self.classIdMovingThings = [1,2,3,4,5,6,7,8,9,16,17,18,19,20,21,22,23,24,25] # Class ID of moving things
        self.classIds = []
        self.boxes = []
        self.image_identified_moving_thing_in_front = False

        self.classNames= []
        self.classFile = 'coco.names'
        with open(self.classFile,'rt') as f:
            self.classNames = f.read().rstrip('\n').split('\n')

        self.configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
        self.weightsPath = 'frozen_inference_graph.pb'

        self.net = cv2.dnn_DetectionModel(self.weightsPath,self.configPath)
        self.net.setInputSize(320,320)
        self.net.setInputScale(1.0/ 127.5)
        self.net.setInputMean((127.5, 127.5, 127.5))
        self.net.setInputSwapRB(True)

        rospy.init_node('image_handler', anonymous=True)
        self.image_raw_compressed_sub = rospy.Subscriber("cv_camera/image_raw/compressed",CompressedImage,self.imgmsg_compressed_callback)
        self.image_identified_frame_compressed_pub = rospy.Publisher("image_identified_frame/compressed",CompressedImage,queue_size=2)
        self.image_identified_boxes_pub = rospy.Publisher("image_identified_boxes",String,queue_size=2)
        self.image_identified_classIds_pub = rospy.Publisher("image_identified_classIds",String,queue_size=2)
        self.image_identified_moving_thing_in_front_pub = rospy.Publisher("image_identified_moving_thing_in_front",Bool,queue_size=2)

        # UNCOMMENT IF WANT TO CHECK IDENTIFICATION RESULT
        self.image_identified_frame_compressed_sub = rospy.Subscriber("image_identified_frame/comressed",CompressedImage,self.imgmsg_identified_compressed_callback)

        # NOT IN USE: compressed image is faster so normal image topics is not in use
        #self.image_raw_sub = rospy.Subscriber("cv_camera/image_raw",Image,self.imgmsg_callback)
        #self.image_identified_frame_pub = rospy.Publisher("image_identified_frame",Image,queue_size=2)
        #self.image_identified_frame_sub = rospy.Subscriber("image_identified_frame",Image,self.imgmsg_identified_callback)
        
        self.rate = rospy.Rate(10)


    def shutdown(self):
        cv2.destroyAllWindows()

    def imgmsg_callback(self,data):
        if len(self.image_raw_frame) > 0:
            return
        try:
            self.image_raw_frame.append(self.bridge.imgmsg_to_cv2(data, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def imgmsg_compressed_callback(self,data):
        if len(self.image_raw_frame) > 0:
            return
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.image_raw_frame.append(image_np)

    def imgmsg_identified_callback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("From Topic: image_identified_frame",img)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)

    def imgmsg_identified_compressed_callback(self,data):
        np_arr = np.fromstring(data.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imshow("From Topic: image_identified_frame_compressed",img)
        cv2.waitKey(3)

    def publish_image_identified_frame(self):
        try:
            img = self.bridge.cv2_to_imgmsg(self.image_identified_frame, encoding='bgr8')
            self.image_identified_frame_pub.publish(img)
        except CvBridgeError as e:
            print(e)

    def publish_image_identified_frame_compressed(self):
        #Create CompressedIamge
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', self.image_identified_frame)[1]).tostring()
        # Publish new image
        self.image_identified_frame_compressed_pub.publish(msg)

    def publish_image_identified_moving_thing_in_front(self):
        msg = Bool()
        msg.data = self.image_identified_moving_thing_in_front
        self.image_identified_moving_thing_in_front_pub.publish(msg)

    def publish_image_identified_info(self):
        boxes = String()
        boxes.data = self.boxes
        classIds = String()
        classIds.data = self.classIds
        self.image_identified_boxes_pub.publish(boxes)
        self.image_identified_classIds_pub.publish(classIds)

    def show_image_raw(self):
        # show black image if there is no image received curently
        if len(self.image_raw_frame) > 0:
            img = self.image_raw_frame[0]
        else: 
            img  = self.image_raw_hold
        cv2.imshow("self.image_raw_frame[0]",img)
        cv2.waitKey(3)

    def show_image_identified_frame(self):
        # show black image if there is no image received curently
        img = self.image_identified_frame
        cv2.imshow("self.image_identified_frame",img)
        cv2.waitKey(3)

    def objectIdentification(self):
        # return of no image received currently
        if len(self.image_raw_frame) > 0:
            img = self.image_raw_frame[0]
        else: 
            print("No image received from webcam.")
            return
        classIdMovingThings = self.classIdMovingThings
        classIds, confs, bbox = self.net.detect(img,confThreshold=self.thres)
        movingThingInFront = False

        if len(classIds) != 0:
            for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
                height, width, channels = img.shape
                boxColor = (255,255,255)
                if classId > 0 and classId < len(self.classNames):
                    if classId in classIdMovingThings:
                        if (((box[0] > width/4) and (box[0] <= width/4*3)) or (((box[0]+box[2]) > width/4) and ((box[0]+box[2]) <= width/4*3)) or 
                            ((box[0] <= width/4) and ((box[0]+box[2]) > width/4*3))):
                            movingThingInFront = True
                            boxColor = (0,0,255)
                        else:
                            boxColor = (255,0,0)
                    cv2.rectangle(img,box,color=boxColor,thickness=2)
                    cv2.putText(img,self.classNames[classId-1].upper(),(box[0]+10,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,boxColor,2)
                    cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,boxColor,2)
                    text = str(box) + str(width)
                    cv2.putText(img,text,(box[0]+10,box[1]+70),
                    cv2.FONT_HERSHEY_COMPLEX,1,boxColor,2)

        self.classIds = str(classIds)
        self.boxes = str(bbox)
        self.image_identified_frame = img
        self.image_identified_moving_thing_in_front = movingThingInFront

        # save last image to self.image_raw_hold
        self.image_raw_hold = self.image_raw_frame[0]
        # clear image_raw_frame to allow new image come in
        self.image_raw_frame = []

        print("classIds | boxes | Moving Thing : ", self.classIds, self.boxes, self.image_identified_moving_thing_in_front)


    def run(self):
        while not rospy.is_shutdown():
            self.objectIdentification()

            self.publish_image_identified_info()
            self.publish_image_identified_moving_thing_in_front()
            self.publish_image_identified_frame_compressed()
            
            # UNCOMMENT TO CHECK image_raw (raw/compress)
            self.show_image_raw()

            # UNCOMMENT TO CHECK identified image
            self.show_image_identified_frame()

            # Not compressed images
            #self.publish_image_identified_frame()

            self.rate.sleep()


if __name__ == '__main__':
    try:
        ih = ImageHandler()
        ih.run()
    except rospy.ROSInterruptException:
        pass