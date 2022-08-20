#!/usr/bin/env python
# -*- coding:utf-8 -*-
import binascii
import math
import serial
import struct
import time
import tf
import rospy
import serial.tools.list_ports
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

cov_orientation = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
cov_angular_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
cov_linear_acceleration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
cov_magnetic_field = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def eul_to_qua(Eular):
    eular_div = [0, 0, 0]
    eular_div[0], eular_div[1], eular_div[2] = Eular[0] / 2.0, Eular[1] / 2.0, Eular[2] / 2.0

    ca, cb, cc = math.cos(eular_div[0]), math.cos(eular_div[1]), math.cos(eular_div[2])
    sa, sb, sc = math.sin(eular_div[0]), math.sin(eular_div[1]), math.sin(eular_div[2])

    x = sa * cb * cc - ca * sb * sc
    y = ca * sb * cc + sa * cb * sc
    z = ca * cb * sc - sa * sb * cc
    w = ca * cb * cc + sa * sb * sc

    orientation = Quaternion()
    orientation.x, orientation.y, orientation.z, orientation.w = x, y, z, w
    return orientation


def receive_split(receive_buffer):
    buff = []
    for i in range(0, len(receive_buffer), 2):
        buff.append(receive_buffer[i:i + 2])
    return buff


def hex_to_ieee(len, buff):
    str = ''
    data = []
    for i in range(len / 2 - 3, 11, -4):
        for j in range(i, i - 4, -1):
            str += buff[j]
        data.append(struct.unpack('>f', str.decode('hex'))[0])
        str = ''
    data.reverse()
    return data


# 查找 ttyUSB* 设备
def find_ttyUSB():
    count = 0
    port_list = []
    for ser in serial.tools.list_ports.comports():
        if str(ser.name).find("USB") == 3:
            print("\033[32m找到了:" + str(ser.name) + " 设备\033[0m")
            port_list.append(str(ser.name))
        else:
            count += 1
            if count == len(list(serial.tools.list_ports.comports())):
                print("\033[31m没有找到相关的 ttyUSB* 设备\033[0m")
                exit(0)
    return port_list


if __name__ == "__main__":
    port_list = find_ttyUSB()
    rospy.init_node("imu")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baudrate = rospy.get_param("~baudrate", 921600)
    try:
        hf_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
        if hf_imu.isOpen():
            rospy.loginfo("\033[32m串口打开成功...\033[0m")
        else:
            hf_imu.open()
            rospy.loginfo("\033[32m打开串口成功...\033[0m")
    except Exception as e:
        print(e)
        rospy.loginfo("\033[31m串口错误，其他因素\033[0m")
        exit(0)
    else:
        imu_pub = rospy.Publisher("handsfree/imu", Imu, queue_size=10)
        mag_pub = rospy.Publisher("handsfree/mag", MagneticField, queue_size=10)
        imu_yaw_degree_pub = rospy.Publisher("handsfree/imu_yaw_degree", Float32, queue_size=10)
        imu_yaw_radian_pub = rospy.Publisher("handsfree/imu_yaw_radian", Float32, queue_size=10)
        
        sensor_data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        data_timeout = 0
        while not rospy.is_shutdown():
            if data_timeout < 1000:
                data_timeout += 1
            else:
                print("\033[31m读取不到 imu 数据，当前 ttyUSB0 设备不是 imu\033[0m")
                exit(0)
            try:
                count = hf_imu.inWaiting()
            except Exception as e:
                print(e)
                print("\033[31mimu 失联\033[0m")
                exit(0)
            else:
                if count > 24:
                    # bytearray() 方法返回一个新字节数组。这个数组里的元素是可变的，并且每个元素的值范围: 0 <= x < 256
                    receive_buffer = bytearray()
                    receive_buffer = binascii.b2a_hex(hf_imu.read(count))
                    receive_len = len(receive_buffer)
                    stamp = rospy.get_rostime()
                    buff = receive_split(receive_buffer)
                    rpy_degree = []
                    if buff[0] + buff[1] + buff[2] == 'aa552c' and len(buff) == 49:
                        sensor_data = hex_to_ieee(len(buff) * 2, buff)
                    if buff[0] + buff[1] + buff[2] == 'aa5514' and len(buff) == 25:
                        data_timeout = 0
                        rpy = hex_to_ieee(len(buff) * 2, buff)
                        rpy_degree.append(rpy[0] / 180 * math.pi)
                        rpy_degree.append(rpy[1] / -180 * math.pi)
                        rpy_degree.append(rpy[2] / -180 * math.pi)

                        imu_msg = Imu()
                        imu_msg.header.stamp = stamp
                        imu_msg.header.frame_id = "imu_link"

                        # 调用 eul_to_qua , 将欧拉角转四元数
                        imu_msg.orientation = eul_to_qua(rpy_degree)
                        imu_msg.orientation_covariance = cov_orientation

                        imu_msg.angular_velocity.x = sensor_data[0]
                        imu_msg.angular_velocity.y = sensor_data[1]
                        imu_msg.angular_velocity.z = sensor_data[2]
                        imu_msg.angular_velocity_covariance = cov_angular_velocity

                        imu_msg.linear_acceleration.x = sensor_data[3] * -9.8
                        imu_msg.linear_acceleration.y = sensor_data[4] * -9.8
                        imu_msg.linear_acceleration.z = sensor_data[5] * -9.8
                        imu_msg.linear_acceleration_covariance = cov_linear_acceleration

                        imu_pub.publish(imu_msg)

                        mag_msg = MagneticField()
                        mag_msg.header.stamp = stamp
                        mag_msg.header.frame_id = "imu_link"
                        mag_msg.magnetic_field.x = sensor_data[6]
                        mag_msg.magnetic_field.y = sensor_data[7]
                        mag_msg.magnetic_field.z = sensor_data[8]
                        mag_msg.magnetic_field_covariance = cov_magnetic_field

                        mag_pub.publish(mag_msg)
                        
                        (r,p,y) = tf.transformations.euler_from_quaternion((imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w))
                        #由于是弧度制，下面将其改成角度制看起来更方便
                        rospy.loginfo("Roll = %f, Pitch = %f, Yaw = %f",r*180/3.1415926,p*180/3.1415926,y*180/3.1415926)
                        Yaw_angle_degree = y*180/math.pi
                        imu_yaw_degree_pub.publish(Yaw_angle_degree)
                        Yaw_angle_radian = y
                        imu_yaw_radian_pub.publish(Yaw_angle_radian)
                        
                        
                        

                time.sleep(0.001)

