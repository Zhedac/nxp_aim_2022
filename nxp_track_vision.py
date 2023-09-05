#!/usr/bin/env python3
# import os
# import sys
import copy
# import re
# import importlib
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
# from rclpy.exceptions import ParameterNotDeclaredException
# from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
import sensor_msgs.msg
from std_msgs.msg import String # will be used in creating a publisher
import nxp_cup_interfaces.msg
from cv_bridge import CvBridge
# from rclpy.qos import QoSProfile
import cv2
if cv2.__version__ < "4.0.0":
    raise ImportError("Requires opencv >= 4.0, "
                      "but found {:s}".format(cv2.__version__))

class NXPTrackVision(Node):

    def __init__(self):

        super().__init__("nxp_track_vision")

        # Get paramaters or defaults
        pyramid_down_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description='Number of times to pyramid image down.')

        camera_image_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Camera image topic.')
        
        debug_image_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Run in debug mode and publish to debug image topic')

        namespace_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Namespaceing if needed.')

        mask_ratio_array_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            description='Array for mask ratio')
        
        self.declare_parameter("pyramid_down", 2, 
            pyramid_down_descriptor)
        
        self.declare_parameter("camera_image", "Pixy2CMUcam5_sensor", 
            camera_image_topic_descriptor)
        
        self.declare_parameter("debug_image", "", 
            debug_image_topic_descriptor)

        self.declare_parameter("namespace", "", 
            namespace_topic_descriptor)

        self.declare_parameter("mask_ratio_array", [1.0, 0.5], 
            mask_ratio_array_descriptor)

        self.pyrDown = self.get_parameter("pyramid_down").value

        self.cameraImageTopic = self.get_parameter("camera_image").value

        self.debugImageTopic = self.get_parameter("debug_image").value

        self.namespaceTopic = self.get_parameter("namespace").value

        self.mask_ratio_array = self.get_parameter("mask_ratio_array").value

        self.imgshape = (936,1296) # size of the image as defined in nxp_cupcar.sdf.jinja
        self.smallsize = (self.imgshape[0]//2,self.imgshape[1]//2) # reducinng image size since we are not processing such large image
        self.widthratio = int(0.35*self.smallsize[1]) # this is the sidewise width to be taken from the image to know the sidewise background(whether its road or not)
        self.lower = np.array([0,0,200]) # array of lower end values of hue, sat and val for white colour segmantation
        self.upper = np.array([179,100,255]) # array of higher end values of hue, sat and val for white colour segmantation
        (self.h,self.w) = self.smallsize
        pts1 = np.array([[0,268],[self.w,268],[514,0],[142,0]],np.float32) # this are the points of the road boundary as viewed by pixy camera(which will be trapezium)
        pts2 = np.array([  [0,self.h], [self.w,self.h],  [self.w,0],  [0,0]],  np.float32)
        self.mat=cv2.getPerspectiveTransform(pts1,pts2) # this is the transformation matrix to transform our image to bird's eye view
        

        #setup CvBridge
        self.bridge = CvBridge()
        
        #Rectangualr area to remove from image calculation to 
        # eliminate the vehicle. Used as ratio of overall image width and height
        # "width ratio,height ratio"
        self.maskRectRatioWidthHeight = np.array([float(self.mask_ratio_array[0]),float(self.mask_ratio_array[1])])
        
        #Bool for generating and publishing the debug image evaluation
        self.debug = False
        
        if self.debugImageTopic != "":
            self.debug = True

        self.timeStamp = self.get_clock().now().nanoseconds
        
        #Pixy image size parameters
        self.pixyImageWidth = 72
        self.pixyImageHeight = 52
        
        #Subscribers
        self.imageSub = self.create_subscription(sensor_msgs.msg.Image, 
            '/{:s}/image_raw'.format(self.cameraImageTopic), 
            self.pixyImageCallback, 
            qos_profile_sensor_data)

        #Publishers
        self.debugDetectionImagePub = self.create_publisher(sensor_msgs.msg.Image,
            '/{:s}'.format(self.debugImageTopic), 0)

        self.imageshape = self.create_publisher(String,"/middle",0) # our subscriber
        self.string = String() # this variable will store our message for subscribing
        
        self.PixyVectorPub = self.create_publisher(nxp_cup_interfaces.msg.PixyVector,
            '{:s}/PixyVector'.format(self.namespaceTopic), 0)
        
        #Only used for debugging line finding issues
        self.lineFindPrintDebug = False
        self.lineMethodsUsedCount = [0, 0, 0, 0, 0, 0, 0]
        self.lineMethodUsed = 0

        (self.pX0, self.pY0, self.pX1, self.pY1) = (0,1,2,3)
        
        #Testing
        self.useBogusData=False
        self.publishSecondVector=True
        self.sortRightToLeft=False
        self.switchVectorPoints=True
        self.stage=0
        self.startTime=0
        self.testAllConfigs=False

        self.bogusOffsetNumber = 3
          
        if self.sortRightToLeft:
            self.bogusDataTest=[[int((self.pixyImageWidth*0.8)+
                self.bogusOffsetNumber),0,
                int(self.pixyImageWidth*0.8),self.pixyImageHeight],
                [int((self.pixyImageWidth*0.2)-self.bogusOffsetNumber),0,
                int(self.pixyImageWidth*0.2),self.pixyImageHeight]]
        else:
            self.bogusDataTest=[[int((self.pixyImageWidth*0.2)+
                self.bogusOffsetNumber),0,
                int(self.pixyImageWidth*0.2),self.pixyImageHeight],
                [int((self.pixyImageWidth*0.8)-self.bogusOffsetNumber),0,
                int(self.pixyImageWidth*0.8),self.pixyImageHeight]]

        if self.switchVectorPoints:
            (self.pX0, self.pY0, self.pX1, self.pY1) = (2,3,0,1)

    def findLines(self, passedImage):
        
        self.timeStamp = self.get_clock().now().nanoseconds
        
        #Testing
        if self.testAllConfigs:
            if self.stage == 0:
                self.get_logger().info("\nPublishing lines Left to Right, points NOT switched")
                self.sortRightToLeft=False
                self.switchVectorPoints=False
                self.startTime = self.timeStamp
                self.stage = 1
            elif (((self.timeStamp - self.startTime) > 30*1e6) and (self.stage == 1)):
                self.sortRightToLeft=False
                self.switchVectorPoints=True
                self.stage = 2
                self.get_logger().info("\nPublishing lines Left to Right, points ARE switched")
            elif (((self.timeStamp - self.startTime) > 40*1e6) and (self.stage == 2)):
                self.sortRightToLeft=True
                self.switchVectorPoints=False
                self.stage = 3
                self.get_logger().info("\nPublishing lines Right to Left, points NOT switched")
            elif (((self.timeStamp - self.startTime) > 50*1e6) and (self.stage == 3)):
                self.sortRightToLeft=True
                self.switchVectorPoints=True
                self.stage = 4
                self.get_logger().info("\nPublishing lines Right to Left, points ARE switched")

        
        #convert image to grayscale
        passedImageGray = cv2.cvtColor(passedImage,cv2.COLOR_BGR2GRAY)
        
        #Image dimensions
        imageHeight, imageWidth = passedImageGray.shape[:2]
        
        #Threshold image black and white
        passedImageGrayThresh = cv2.bitwise_not(cv2.threshold(
            passedImageGray, 80, 255, cv2.THRESH_BINARY)[1])
        
        #Create image mask background
        maskWhite = np.ones(passedImageGrayThresh.shape[:2], dtype="uint8") * 255
        
        #calculate points to be masked based on provided ratio
        maskVehicleBoxTopLeftXY = (int(imageWidth*(1.0-self.maskRectRatioWidthHeight[0])/2.0), 
            int(imageHeight*(1.0-self.maskRectRatioWidthHeight[1])))
        
        #calculate points to be masked based on provided ratio
        maskVehicleBoxBottomRightXY = (int(imageWidth*(1.0+self.maskRectRatioWidthHeight[0])/2.0), 
            int(imageHeight))
        
        maskVehicle = cv2.rectangle(maskWhite,maskVehicleBoxTopLeftXY,
            maskVehicleBoxBottomRightXY,color=0,thickness=-1)
        
        #Mask out the area of the vehicle
        passedImageGrayThreshMasked = cv2.bitwise_and(passedImageGrayThresh, 
            passedImageGrayThresh, mask=maskVehicle)
        
        #Find contours
        cnts, hierarchy = cv2.findContours(passedImageGrayThreshMasked.copy(),
            cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        returnedImageDebug=passedImage

        #Max number of found contours to process based on area of return, largest returned first
        maxCnt = 2
        if len (cnts) < maxCnt:
            maxCnt = len(cnts)
        cnt = sorted(cnts, key=cv2.contourArea, reverse=True)[0:maxCnt]

        #Take largest contours and sort left to right
        if len (cnts) > 1:
            boundingBoxes = [cv2.boundingRect(c) for c in cnt]
            (cnt, boundingBoxes) = zip(*sorted(zip(cnt, boundingBoxes),
                key=lambda b:b[1][0], reverse=self.sortRightToLeft))

        #Initialize and/or clear existing found line vector array
        pixyScaledVectorArray = np.empty([0,4], int)

        #Used to determine what line is mapped in message from debug image
        lineNumber=0

        #Loop through contours
        for cn in cnt:

            if self.debug:
                #Paint all the areas found in the contour
                cv2.fillPoly(returnedImageDebug,pts=[cn],color=(0,0,255))
            
            #Find lines from contours using least square method
            [vectorX,vectorY,linePointX,linePointY] = cv2.fitLine(cn,cv2.DIST_L2,0,0.01,0.01)
            
            if (linePointX >= 1.0) and (linePointY >= 1.0):
                #Easy avoid divide by zero
                if vectorY == 0:
                    vectorY = 1e-4
                if vectorX == 0:
                    vectorX = 1e-4

                #Calculate line points to see if they exceeds any bounds of the image, correct if they do
                topLeftX = int((-linePointY*vectorX/vectorY)+linePointX)
                bottomRightX = int(((imageHeight-linePointY)*vectorX/vectorY)+linePointX)
                topLeftY = 0
                bottomRightY = imageHeight
                self.lineMethodUsed = 0
                
                if (topLeftX <= 0) and (bottomRightX > imageWidth):
                    self.lineMethodUsed = 1
                    topLeftY = int(((-linePointX)*vectorY/vectorX)+linePointY)
                    bottomRightY = int(((imageWidth-linePointX)*vectorY/vectorX)+linePointY)
                    topLeftX = 0
                    bottomRightX = imageWidth

                elif (topLeftX > imageWidth) and (bottomRightX < 0):
                    self.lineMethodUsed = 2
                    topLeftY = int(((imageWidth-linePointX)*vectorY/vectorX)+linePointY)
                    bottomRightY = int(((-linePointX)*vectorY/vectorX)+linePointY)
                    topLeftX = imageWidth
                    bottomRightX = 0

                elif (topLeftX <= 0) and (bottomRightX < imageWidth):
                    self.lineMethodUsed = 3
                    topLeftY = int(((-linePointX)*vectorY/vectorX)+linePointY)
                    bottomRightY = imageHeight
                    topLeftX = 0
                    bottomRightX = int(((imageHeight-linePointY)*vectorX/vectorY)+linePointX)

                elif (topLeftX > 0) and (bottomRightX > imageWidth):
                    self.lineMethodUsed = 4
                    topLeftY = 0
                    bottomRightY = int(((imageWidth-linePointX)*vectorY/vectorX)+linePointY)
                    topLeftX = int((-linePointY*vectorX/vectorY)+linePointX)
                    bottomRightX = imageWidth

                elif (topLeftX > imageWidth) and (bottomRightX > 0):
                    self.lineMethodUsed = 5
                    topLeftY = imageHeight
                    bottomRightY = int(((imageWidth-linePointX)*vectorY/vectorX)+linePointY)
                    topLeftX = int(((imageHeight-linePointY)*vectorX/vectorY)+linePointX)
                    bottomRightX = imageWidth

                elif (topLeftX < imageWidth) and (bottomRightX < 0):
                    self.lineMethodUsed = 6
                    topLeftY = int((-linePointX*vectorY/vectorX)+linePointY)
                    bottomRightY = 0
                    topLeftX = 0
                    bottomRightX = int((linePointY)*(-vectorX/vectorY)+linePointX)
                
                
                #Extra safety
                if topLeftX < 0:
                    topLeftX=0
                elif topLeftX > imageWidth:
                    topLeftX=imageWidth
                if bottomRightX < 0:
                    bottomRightX=0
                elif bottomRightX > imageWidth:
                    bottomRightX=imageWidth
                if topLeftY < 0:
                    topLeftY=0
                elif topLeftY > imageHeight:
                    topLeftY=imageHeight
                if bottomRightY < 0:
                    bottomRightY=0
                elif bottomRightY > imageHeight:
                    bottomRightY=imageHeight

                #Add line method used to array count
                self.lineMethodsUsedCount[self.lineMethodUsed]+=1
                
                #Scale into Pixy camera units
                topLeftXScaled = int(topLeftX*(self.pixyImageWidth/imageWidth))
                bottomRightXScaled = int(bottomRightX*(self.pixyImageWidth/imageWidth))
                topLeftYScaled = int(topLeftY*(self.pixyImageWidth/imageWidth))
                bottomRightYScaled = int(bottomRightY*(self.pixyImageWidth/imageWidth))

                if self.lineFindPrintDebug:
                    self.get_logger().info('\n\nlineMethodUsed:{:d},lineMethodsUsedCount:{:s}'.format(
                        self.lineMethodUsed,str(self.lineMethodsUsedCount)))
                    self.get_logger().info('vectorX:{:f},vectorY:{:f},linePointX:{:f},linePointY:{:f}'.format(
                        float(vectorX),float(vectorY),float(linePointX),float(linePointY)))
                    self.get_logger().info('topLeftX:{:d},topLeftY:{:d},bottomRightX:{:d},bottomRightY:{:d}'.format(
                        topLeftX,topLeftY,bottomRightX,bottomRightY))
                    returnedImageDebug = cv2.line(returnedImageDebug,(topLeftX,topLeftY),
                        (bottomRightX,bottomRightY),(255,128,128),2)

                #Append found line points to pixy found line vector array
                pixyScaledVectorArray = np.append(pixyScaledVectorArray,
                    np.array([[topLeftXScaled,topLeftYScaled,bottomRightXScaled,bottomRightYScaled]]),axis=0)
                #Increment line number
                lineNumber += 1
        
        #Testing
        if self.useBogusData:
            if self.sortRightToLeft:
                self.bogusDataTest=[[int((self.pixyImageWidth*0.8)+
                    self.bogusOffsetNumber),0,
                    int(self.pixyImageWidth*0.8),self.pixyImageHeight],
                    [int((self.pixyImageWidth*0.2)-self.bogusOffsetNumber),0,
                    int(self.pixyImageWidth*0.2),self.pixyImageHeight]]
            else:
                self.bogusDataTest=[[int((self.pixyImageWidth*0.2)+
                    self.bogusOffsetNumber),0,
                    int(self.pixyImageWidth*0.2),self.pixyImageHeight],
                    [int((self.pixyImageWidth*0.8)-self.bogusOffsetNumber),0,
                    int(self.pixyImageWidth*0.8),self.pixyImageHeight]]

            if self.switchVectorPoints:
                (self.pX0, self.pY0, self.pX1, self.pY1) = (2,3,0,1)
            else:
                (self.pX0, self.pY0, self.pX1, self.pY1) = (0,1,2,3)

            pixyScaledVectorArray = np.array(self.bogusDataTest)

        if self.debug:
            #Border for vehicle mask
            returnedImageDebug = cv2.rectangle(returnedImageDebug,maskVehicleBoxTopLeftXY,
                maskVehicleBoxBottomRightXY, color=(128,128,0),thickness=3)

            #Calcualte background for pixy image space
            debugPixyMessageTopLeftXY = (int((imageWidth/2)-(self.pixyImageWidth/2)),
                int((imageHeight/2)-(self.pixyImageHeight/2)))

            #Create background for pixy image space
            returnedImageDebug = cv2.rectangle(returnedImageDebug,debugPixyMessageTopLeftXY,
                ((debugPixyMessageTopLeftXY[0]+self.pixyImageWidth),
                (debugPixyMessageTopLeftXY[1]+self.pixyImageHeight)),(0,255,0),-1)
            
            for lineNumber in range(len(pixyScaledVectorArray)):
                #Draw the found line in image space
                returnedImageDebug = cv2.line(returnedImageDebug,
                    (debugPixyMessageTopLeftXY[0]+pixyScaledVectorArray[lineNumber][0],
                    debugPixyMessageTopLeftXY[1]+pixyScaledVectorArray[lineNumber][1]),
                    (debugPixyMessageTopLeftXY[0]+pixyScaledVectorArray[lineNumber][2],
                    debugPixyMessageTopLeftXY[1]+pixyScaledVectorArray[lineNumber][3]),
                    (255,128,128),1)
                
                #Draw box point for top left XY for Pixy space debug image
                returnedImageDebug = cv2.rectangle(returnedImageDebug,
                    (debugPixyMessageTopLeftXY[0]+pixyScaledVectorArray[lineNumber][0]-1, 
                    debugPixyMessageTopLeftXY[1]+pixyScaledVectorArray[lineNumber][1]-1),
                    (debugPixyMessageTopLeftXY[0]+pixyScaledVectorArray[lineNumber][0]+1, 
                    debugPixyMessageTopLeftXY[1]+pixyScaledVectorArray[lineNumber][1]+1),
                    (0,0,255),-1)
                
                #Draw box point for bottom right XY for Pixy space debug image
                returnedImageDebug = cv2.rectangle(returnedImageDebug,
                    (debugPixyMessageTopLeftXY[0]+pixyScaledVectorArray[lineNumber][2]-1,
                    debugPixyMessageTopLeftXY[1]+pixyScaledVectorArray[lineNumber][3]-1),
                    (debugPixyMessageTopLeftXY[0]+pixyScaledVectorArray[lineNumber][2]+1, 
                    debugPixyMessageTopLeftXY[1]+pixyScaledVectorArray[lineNumber][3]+1),
                    (0,0,255),-1)
                
                #Write text for top left XY for Pixy space debug image
                returnedImageDebug = cv2.putText(
                    returnedImageDebug, '{:d},{:d}'.format(
                    pixyScaledVectorArray[lineNumber][0],pixyScaledVectorArray[lineNumber][1]), 
                    (debugPixyMessageTopLeftXY[0]+pixyScaledVectorArray[lineNumber][0]-20+(20*lineNumber), 
                    debugPixyMessageTopLeftXY[1]+pixyScaledVectorArray[lineNumber][1]), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255,0,0), 1, cv2.LINE_AA)
                
                #Write text for bottom right XY for Pixy space debug image
                returnedImageDebug = cv2.putText(returnedImageDebug, '{:d},{:d}'.format(
                    pixyScaledVectorArray[lineNumber][2],pixyScaledVectorArray[lineNumber][3]),
                    (debugPixyMessageTopLeftXY[0]+pixyScaledVectorArray[lineNumber][2]-20+(20*lineNumber), 
                    debugPixyMessageTopLeftXY[1]+pixyScaledVectorArray[lineNumber][3]), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255,0,0), 1, cv2.LINE_AA)

                #Write the line number for message
                returnedImageDebug = cv2.putText(returnedImageDebug, 'm{:d}'.format(
                    lineNumber),
                    (int((pixyScaledVectorArray[lineNumber][0]+
                    pixyScaledVectorArray[lineNumber][2])/2.0+
                    debugPixyMessageTopLeftXY[0]-20+(20*lineNumber)),
                    int((pixyScaledVectorArray[lineNumber][1]+
                    pixyScaledVectorArray[lineNumber][3])/2.0+
                    debugPixyMessageTopLeftXY[1])), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,0,255), 1, cv2.LINE_AA)

        #Pixy message for publication
        if (len(pixyScaledVectorArray) == 0):
            PixyVector_msg = nxp_cup_interfaces.msg.PixyVector()
            PixyVector_msg.timestamp = int(self.timeStamp)
            PixyVector_msg.m0_x0 = int(0)
            PixyVector_msg.m0_y0 = int(0)
            PixyVector_msg.m0_x1 = int(0)
            PixyVector_msg.m0_y1 = int(0)
            PixyVector_msg.m1_x0 = int(0)
            PixyVector_msg.m1_y0 = int(0)
            PixyVector_msg.m1_x1 = int(0)
            PixyVector_msg.m1_y1 = int(0)
            self.PixyVectorPub.publish(PixyVector_msg)
        if (len(pixyScaledVectorArray) > 0):
            PixyVector_msg = nxp_cup_interfaces.msg.PixyVector()
            PixyVector_msg.timestamp = int(self.timeStamp)
            PixyVector_msg.m0_x0 = int(pixyScaledVectorArray[0][self.pX0])
            PixyVector_msg.m0_y0 = int(pixyScaledVectorArray[0][self.pY0])
            PixyVector_msg.m0_x1 = int(pixyScaledVectorArray[0][self.pX1])
            PixyVector_msg.m0_y1 = int(pixyScaledVectorArray[0][self.pY1])
            PixyVector_msg.m1_x0 = int(0)
            PixyVector_msg.m1_y0 = int(0)
            PixyVector_msg.m1_x1 = int(0)
            PixyVector_msg.m1_y1 = int(0)
            if (len(pixyScaledVectorArray) > 1) and self.publishSecondVector:
                PixyVector_msg.m1_x0 = int(pixyScaledVectorArray[1][self.pX0])
                PixyVector_msg.m1_y0 = int(pixyScaledVectorArray[1][self.pY0])
                PixyVector_msg.m1_x1 = int(pixyScaledVectorArray[1][self.pX1])
                PixyVector_msg.m1_y1 = int(pixyScaledVectorArray[1][self.pY1])
            self.PixyVectorPub.publish(PixyVector_msg)

        return(returnedImageDebug)
      
    
    def pixyImageCallback(self, data):
        
        # Scene from subscription callback
        scene = self.bridge.imgmsg_to_cv2(data, "bgr8")

        #deep copy and pyramid down image to reduce resolution
        scenePyr = copy.deepcopy(scene)
        if self.pyrDown > 0:
            for i in range(self.pyrDown):
                scenePyr = cv2.pyrDown(scenePyr)
        sceneDetect = copy.deepcopy(scenePyr)

        #find lines function
        sceneDetected = self.findLines(sceneDetect)
        
        if self.debug:
            #publish debug image
            msg = self.bridge.cv2_to_imgmsg(sceneDetected, "bgr8")
            msg.header.stamp = data.header.stamp
            self.debugDetectionImagePub.publish(msg)
        threshold = 210
        smallimage = cv2.resize(scene,(self.smallsize[1],self.smallsize[0])) # grabbing our image(resizing it since will not use that large image)
        hsv = cv2.cvtColor(smallimage,cv2.COLOR_BGR2HSV) # converting to hsv
        mask = cv2.inRange(hsv,self.lower,self.upper) # creating mask of our image for white colour(road segmentation)
        transform=cv2.warpPerspective(mask,self.mat,(self.w,self.h)) # transforming to bird's eye view
        left = transform[:,:self.widthratio] # grabbing left most region of our image
        right = transform[:,self.smallsize[1]-self.widthratio:] # grabbing right most region of our image
        # cv2.imshow("mask",mask)
        # cv2.imshow("transform",transform)
        # cv2.waitKey(5)
        left = np.where(left <= threshold, 0, 1) # we want to know how many white pixels are there(how much area is under road)
        right = np.where(right <= threshold, 0, 1) # we want to know how many white pixels are there(how much area is under road)
        s = (left.shape[0])*(left.shape[1]) # total no. of pixels
        self.string.data = f"{np.sum(left)/s}%{np.sum(right)/s}" # encripting the average white pixel density of left and right side to our message
        self.imageshape.publish(self.string) # publishing message

def main(args=None):
    rclpy.init(args=args)
    node = NXPTrackVision()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
