#!/usr/bin/env python
# -*- coding:utf-8 -*-
import math
import serial
import struct
import time

# 在缓冲数据中找到第一个包的起始位置
def find_first_package(buffer):
    i = 0
    while True:
        if buffer[i] == 0x55 and (buffer[i + 1] & 0x50) == 0x50:
            return i
        if i + 2 >= len(buffer):
            return -1
        i += 1


# 检查校验和
def sb_sum_chech(byte_temp):
    # if(len(byte_temp)==11) :
    if (((byte_temp[0] + byte_temp[1] + byte_temp[2] + byte_temp[3] + byte_temp[4] + byte_temp[5] + byte_temp[6] +
          byte_temp[7] + byte_temp[8] + byte_temp[9]) & 0xff) == byte_temp[10]):
        # print('sum check ok!')
        return True
    else:
        # print('sum check false!')
        return False


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
    try:
        hf_imu = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=0.5)
        if hf_imu.isOpen():
            print("\033[32m串口打开成功...\033[0m")
        else:
            hf_imu.open()
            print("\033[32m打开串口成功...\033[0m")
    except Exception as e:
        print(e)
        print("\033[31m串口错误，其他因素\033[0m")
        exit(0)
    else:
        receive_buffer = bytearray()
        linear_acceleration_x = 0
        linear_acceleration_y = 0
        linear_acceleration_z = 0
        angular_velocity_x = 0
        angular_velocity_y = 0
        angular_velocity_z = 0
        data_timeout = 0
        while True:
            if data_timeout < 1000:
                data_timeout += 1
            else:
                print("\033[31m读取不到 imu 数据，当前 ttyUSB0 设备不是 imu\033[0m")
                exit(0)
            eul = []
            try:
                count = hf_imu.inWaiting()
            except Exception as e:
                print(e)
                print("\033[31mimu 失联\033[0m")
                exit(0)
            else:
                if count > 0:
                    s = hf_imu.read(count)
                    receive_buffer += s

                dataLen = len(receive_buffer)
                if dataLen >= 11:
                    # 去掉第1个包头前的数据
                    headerPos = find_first_package(receive_buffer)
                    if headerPos >= 0:
                        if headerPos > 0:
                            receive_buffer[0:headerPos] = b''
                        # 取 Config.minPackageLen 整数倍长度的数据
                        if dataLen - headerPos >= 11:
                            packageCount = int((dataLen - headerPos) / 11)
                            if packageCount > 0:
                                cutLen = packageCount * 11
                                temp = receive_buffer[0:cutLen]
                                # 按16进制字符串的形式显示收到的内容
                                receive_buffer[0:cutLen] = b''

                                # 解析数据,逐个数据包进行解析
                                for i in range(packageCount):
                                    beginIdx = int(i * 11)
                                    endIdx = int(i * 11 + 11)
                                    byte_temp = temp[beginIdx:endIdx]
                                    # 校验和通过了的数据包才进行解析

                                    if sb_sum_chech(byte_temp):
                                        Data = list(struct.unpack("hhhh", byte_temp[2:10]))
                                        # 加速度
                                        if byte_temp[1] == 0x51:
                                            linear_acceleration_x = Data[0] / 32768.0 * 16 * -9.8
                                            linear_acceleration_y = Data[1] / 32768.0 * 16 * -9.8
                                            linear_acceleration_z = Data[2] / 32768.0 * 16 * -9.8

                                        # 角速度
                                        if byte_temp[1] == 0x52:
                                            angular_velocity_x = Data[0] / 32768.0 * 2000 * math.pi / 180
                                            angular_velocity_y = Data[1] / 32768.0 * 2000 * math.pi / 180
                                            angular_velocity_z = Data[2] / 32768.0 * 2000 * math.pi / 180

                                        # 姿态角
                                        if byte_temp[1] == 0x53:
                                            angle_x = Data[0] / 32768.0 * 180
                                            angle_y = Data[1] / 32768.0 * 180
                                            angle_z = Data[2] / 32768.0 * 180

                                            print('加速度:')
                                            print('\t x轴加速度：' + "%.2f g" % linear_acceleration_x)
                                            print('\t y轴加速度：' + "%.2f g" % linear_acceleration_y)
                                            print('\t z轴加速度：' + "%.2f g" % linear_acceleration_z + "\r\n")

                                            print('角速度：')
                                            print('\t x轴角速度：' + "%.2f °/s" % angular_velocity_x)
                                            print('\t y轴角速度：' + "%.2f °/s" % angular_velocity_y)
                                            print('\t z轴角速度：' + "%.2f °/s" % angular_velocity_z + "\r\n")

                                            print('角度：')
                                            print('\t x轴角度：' + "%.2f °" % angle_x)
                                            print('\t y轴角度：' + "%.2f °" % angle_y)
                                            print('\t z轴角度：' + "%.2f °" % angle_z + "\r\n")
                time.sleep(0.001)

