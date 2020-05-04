import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
import cv2
import matplotlib.pyplot as plt
import numpy as np

class ImageConverter(object):

    def __init__(self):
        rospy.init_node('ImageConverter') 
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/carla/ego_vehicle/camera/rgb/front/image_color", Image, self.callback)
        self.image_pub = rospy.Publisher('/detected', Image, queue_size=1)
        self.det_pub = rospy.Publisher('/machine_learning/output', Int16,queue_size=1)
        rospy.spin()

    def callback(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        objects = self.detect_objects(img)
        #cv2.imshow('Output', objects)
        #cv2.waitKey(1)
        self.cv2pub(objects, "bgr8")

    def cv2pub(self, frame, encode_type):
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encode_type))
        except CvBridgeError as e:
            print(e)

    def detect_objects(self, img):
        # Load Yolo
        net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
        classes = []
        with open("coco.names", "r") as f:
            classes = [line.strip() for line in f.readlines()]
        layer_names = net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        colors = np.random.uniform(0, 255, size=(len(classes), 3))
        c_threshold = 0.3

        width = img.shape[1]
        height = img.shape[0]
        img_nn = cv2.resize(img, None, fx=0.4, fy=0.4)

        # Detecting objects
        blob = cv2.dnn.blobFromImage(img_nn, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        # Showing information on the screen
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > c_threshold:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, c_threshold, 0.4)

        font = cv2.FONT_HERSHEY_SIMPLEX
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                color = colors[i]
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, label, (x, y - 30), font, 1, color, 3)
        try:
            self.det_pub.publish(class_ids[0])
        except:
            pass
        return img

def main():
    ic = ImageConverter()
    #rospy.init_node('ImageConverter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
