# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gps_test.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from __future__ import print_function

import math
import os,sys
import rospy
import sensor_msgs.msg 
import geometry_msgs.msg
import tf
print(sys.version)
import warnings
warnings.filterwarnings('ignore')


from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import QuaternionStamped


import numpy as np
import math
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import *

global gps_datum_data
gps_datum_data=[0.0,0.0,0.0]
global gps_fix_data
gps_fix_data=[0.0,0.0,0.0]

def gps_fix_pub(self):
    global gps_fix_data
    pub = rospy.Publisher("fix", NavSatFix, queue_size=10)
    msg = NavSatFix()
    
    msg.header.stamp = rospy.Time.now()   
    msg.header.frame_id = 'gps'
    msg.latitude = gps_fix_data[0]
    msg.longitude = gps_fix_data[1]
    msg.altitude = 0
    msg.status.status = 2
    msg.position_covariance = (0, 0, 0, 0, 0, 0, 0, 0, 0)
    msg.position_covariance_type = 0
    
    pub.publish(msg)
    
def gps_heading_pub(self):
    global gps_fix_data
    pub = rospy.Publisher("heading", QuaternionStamped, queue_size=10)
    
    
    msg = QuaternionStamped()
    msg.header.stamp = rospy.Time.now()   
    msg.header.frame_id = 'gps'
    
    quaternion = tf.transformations.quaternion_from_euler(0, 0, gps_fix_data[2]/180.0*math.pi)
    msg.quaternion.x = quaternion[0]
    msg.quaternion.y = quaternion[1]
    msg.quaternion.z = quaternion[2]
    msg.quaternion.w = quaternion[3]
    
    pub.publish(msg)    
    

class Ui_Dialog(object):
	
    
     
    def __init__(self):
        rospy.init_node('ROSgui', anonymous=True)
        #GPS datum Subscriber
        rospy.Subscriber('/gps/datum',Vector3, self.subGPSdatumCallback)        

    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(615, 421)
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setGeometry(QtCore.QRect(190, 370, 341, 32))
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.groupBox_datum = QtWidgets.QGroupBox(Dialog)
        self.groupBox_datum.setGeometry(QtCore.QRect(40, 30, 491, 81))
        self.groupBox_datum.setObjectName("groupBox_datum")
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.groupBox_datum)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(10, 41, 471, 31))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.textEdit_datum_latitude = QtWidgets.QTextEdit(self.horizontalLayoutWidget)
        self.textEdit_datum_latitude.setObjectName("textEdit_datum_latitude")
        self.horizontalLayout.addWidget(self.textEdit_datum_latitude)
        self.textEdit_datum_longitude = QtWidgets.QTextEdit(self.horizontalLayoutWidget)
        self.textEdit_datum_longitude.setObjectName("textEdit_datum_longitude")
        self.horizontalLayout.addWidget(self.textEdit_datum_longitude)
        self.textEdit_datum_yaw = QtWidgets.QTextEdit(self.horizontalLayoutWidget)
        self.textEdit_datum_yaw.setObjectName("textEdit_datum_yaw")
        self.horizontalLayout.addWidget(self.textEdit_datum_yaw)
        self.pushButton_datum = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.pushButton_datum.setObjectName("pushButton_datum")
        self.horizontalLayout.addWidget(self.pushButton_datum)
        self.gridLayoutWidget = QtWidgets.QWidget(Dialog)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(40, 190, 491, 131))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.pushButton_move_up = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_move_up.setObjectName("pushButton_move_up")
        self.gridLayout.addWidget(self.pushButton_move_up, 0, 1, 1, 1)
        self.pushButton_move_right = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_move_right.setObjectName("pushButton_move_right")
        self.gridLayout.addWidget(self.pushButton_move_right, 1, 2, 1, 1)
        self.pushButton_move_down = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_move_down.setObjectName("pushButton_move_down")
        self.gridLayout.addWidget(self.pushButton_move_down, 2, 1, 1, 1)
        self.pushButton_move_left = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_move_left.setObjectName("pushButton_move_left")
        self.gridLayout.addWidget(self.pushButton_move_left, 1, 0, 1, 1)
        self.horizontalSlider = QtWidgets.QSlider(Dialog)
        self.horizontalSlider.setGeometry(QtCore.QRect(40, 340, 491, 16))
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.horizontalLayoutWidget_2 = QtWidgets.QWidget(Dialog)
        self.horizontalLayoutWidget_2.setGeometry(QtCore.QRect(50, 120, 381, 31))
        self.horizontalLayoutWidget_2.setObjectName("horizontalLayoutWidget_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_2)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.textEdit_gps_latitude = QtWidgets.QTextEdit(self.horizontalLayoutWidget_2)
        self.textEdit_gps_latitude.setObjectName("textEdit_gps_latitude")
        self.horizontalLayout_2.addWidget(self.textEdit_gps_latitude)
        self.textEdit_gps_longitude = QtWidgets.QTextEdit(self.horizontalLayoutWidget_2)
        self.textEdit_gps_longitude.setObjectName("textEdit_gps_longitude")
        self.horizontalLayout_2.addWidget(self.textEdit_gps_longitude)
        self.textEdit_gps_yaw = QtWidgets.QTextEdit(self.horizontalLayoutWidget_2)
        self.textEdit_gps_yaw.setObjectName("textEdit_gps_yaw")
        self.horizontalLayout_2.addWidget(self.textEdit_gps_yaw)

        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept)
        self.buttonBox.rejected.connect(Dialog.reject)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "MGI-2000 GPS Simulator"))
        self.groupBox_datum.setTitle(_translate("Dialog", "GPS Datum Setup"))
        self.pushButton_datum.setText(_translate("Dialog", "GPS datum"))
        self.pushButton_move_up.setText(_translate("Dialog", "move north"))
        self.pushButton_move_right.setText(_translate("Dialog", "move east"))
        self.pushButton_move_down.setText(_translate("Dialog", "move south"))
        self.pushButton_move_left.setText(_translate("Dialog", "move west"))
        
        
        
        #self.textEdit_datum_latitude.setText("37.30476631")
        #self.textEdit_datum_longitude.setText("127.9075526")
        #self.textEdit_datum_yaw.setText("0")
        
        
        self.textEdit_gps_latitude.setText("37.304163")
        self.textEdit_gps_longitude.setText("127.907548")
        self.textEdit_gps_yaw.setText("0")
        
        
        self.pushButton_datum.clicked.connect(self.datum_button_Clicked)
        self.pushButton_move_up.clicked.connect(self.gps_move_up_button_Clicked)
        self.pushButton_move_down.clicked.connect(self.gps_move_down_button_Clicked)
        self.pushButton_move_right.clicked.connect(self.gps_move_right_button_Clicked)
        self.pushButton_move_left.clicked.connect(self.gps_move_left_button_Clicked)
    
        self.horizontalSlider.setRange(-180,180)
        self.horizontalSlider.setTickInterval(10)
        self.horizontalSlider.valueChanged.connect(self.slider_value_changed);
    
    def subGPSdatumCallback(self,gps_datum_msg):
		#print(gps_datum_msg.x, gps_datum_msg.y ,gps_datum_msg.z)
		global gps_datum_data
		gps_datum_data=[gps_datum_msg.x , gps_datum_msg.y ,gps_datum_msg.z]
		
		#print(gps_datum_data)
		
    def datum_button_Clicked(self):
        global gps_datum_data
        global gps_fix_data
        #print(gps_datum_data)				
        self.textEdit_datum_latitude.setText(str(gps_datum_data[0]))
        self.textEdit_datum_longitude.setText(str(gps_datum_data[1]))
        self.textEdit_datum_yaw.setText(str(gps_datum_data[2]))
        gps_fix_data = [gps_datum_data[0],gps_datum_data[1],gps_datum_data[2]]
        
    def gps_move_up_button_Clicked(self):
        #print("test")
        global gps_fix_data
        
        txt = self.textEdit_gps_latitude.toPlainText()
        gps_fix_data[0] = float(txt);
        txt = self.textEdit_gps_longitude.toPlainText()
        gps_fix_data[1] = float(txt);
        txt = self.textEdit_gps_yaw.toPlainText()
        gps_fix_data[2] = float(txt);
        
        gps_fix_data[1] =  gps_fix_data[1] + 0.000001  #north
        
        self.textEdit_gps_latitude.setText(str(gps_fix_data[0]))
        self.textEdit_gps_longitude.setText(str(gps_fix_data[1]))
        self.textEdit_gps_yaw.setText(str(gps_fix_data[2]))
        gps_fix_pub(self)

    def gps_move_down_button_Clicked(self):
        #print("test")
        global gps_fix_data
        
        txt = self.textEdit_gps_latitude.toPlainText()
        gps_fix_data[0] = float(txt);
        txt = self.textEdit_gps_longitude.toPlainText()
        gps_fix_data[1] = float(txt);
        txt = self.textEdit_gps_yaw.toPlainText()
        gps_fix_data[2] = float(txt);
        
        gps_fix_data[1] =  gps_fix_data[1] - 0.000001   #south
        
        self.textEdit_gps_latitude.setText(str(gps_fix_data[0]))
        self.textEdit_gps_longitude.setText(str(gps_fix_data[1]))
        self.textEdit_gps_yaw.setText(str(gps_fix_data[2]))
        gps_fix_pub(self)
        
    def gps_move_right_button_Clicked(self):
        #print("test")
        global gps_fix_data
        
        txt = self.textEdit_gps_latitude.toPlainText()
        gps_fix_data[0] = float(txt);
        txt = self.textEdit_gps_longitude.toPlainText()
        gps_fix_data[1] = float(txt);
        txt = self.textEdit_gps_yaw.toPlainText()
        gps_fix_data[2] = float(txt);
        
        gps_fix_data[0] =  gps_fix_data[0] + 0.000001     #east
        
        self.textEdit_gps_latitude.setText(str(gps_fix_data[0]))
        self.textEdit_gps_longitude.setText(str(gps_fix_data[1]))
        self.textEdit_gps_yaw.setText(str(gps_fix_data[2]))
        gps_fix_pub(self) 
    
    def gps_move_left_button_Clicked(self):
        #print("test")
        global gps_fix_data
        
        txt = self.textEdit_gps_latitude.toPlainText()
        gps_fix_data[0] = float(txt);
        txt = self.textEdit_gps_longitude.toPlainText()
        gps_fix_data[1] = float(txt);
        txt = self.textEdit_gps_yaw.toPlainText()
        gps_fix_data[2] = float(txt);
        
        gps_fix_data[0] =  gps_fix_data[0] - 0.000001   #west
        
        self.textEdit_gps_latitude.setText(str(gps_fix_data[0]))
        self.textEdit_gps_longitude.setText(str(gps_fix_data[1]))
        self.textEdit_gps_yaw.setText(str(gps_fix_data[2]))
        gps_fix_pub(self)         
    
    
    def slider_value_changed(self,value):
		global gps_fix_data
		gps_fix_data[2] = value
		self.textEdit_gps_yaw.setText(str(gps_fix_data[2]))
		print("slider",value)
		gps_heading_pub(self)

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())

