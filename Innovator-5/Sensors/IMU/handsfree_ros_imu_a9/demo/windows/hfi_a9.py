#!/usr/bin/env python
# -*- coding:utf-8 -*-
import binascii
import math
import serial
import struct
import time
import sys
if sys.getdefaultencoding() != 'utf-8':
    reload(sys)
    sys.setdefaultencoding('utf-8')

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


if __name__ == "__main__":

    try:
        hf_imu = serial.Serial(port='COM3', baudrate=921600, timeout=0.5)
        if hf_imu.isOpen():
            print(u"串口打开成功...")
        else:
            hf_imu.open()
            print(u"打开串口成功...")
    except Exception as e:
        print(e)
        print(u"串口错误，其他因素")
        exit(0)
    else:
        sensor_data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	data_timeout = 0
        while True:
            if data_timeout < 1000:
                data_timeout += 1
            else:
                print(u"读取不到 imu 数据，当前 port 设备不是 imu")
                exit(0)
            try:
                count = hf_imu.inWaiting()
            except Exception as e:
                print(e)
                print(u"imu 失联")
                exit(0)
            else:
                if count > 24:
                    # bytearray() 方法返回一个新字节数组。这个数组里的元素是可变的，并且每个元素的值范围: 0 <= x < 256
                    receive_buffer = bytearray()
                    receive_buffer = binascii.b2a_hex(hf_imu.read(count))
                    receive_len = len(receive_buffer)
                    buff = receive_split(receive_buffer)
                    if buff[0] + buff[1] + buff[2] == 'aa552c' and len(buff) == 49:
                        sensor_data = hex_to_ieee(len(buff) * 2, buff)
                    if buff[0] + buff[1] + buff[2] == 'aa5514' and len(buff) == 25:
                        data_timeout = 0
                        rpy = hex_to_ieee(len(buff) * 2, buff)

                        print(u'加速度:')
                        print(u'\t x轴加速度：' + "%.2f g" % (sensor_data[3] * -9.8))
                        print(u'\t y轴加速度：' + "%.2f g" % (sensor_data[4] * -9.8))
                        print(u'\t z轴加速度：' + "%.2f g" % (sensor_data[5] * -9.8) + "\r\n")

                        print(u'角速度：')
                        print(u'\t x轴角速度：' + "%.2f rad/s" % sensor_data[0])
                        print(u'\t y轴角速度：' + "%.2f rad/s" % sensor_data[1])
                        print(u'\t z轴角速度：' + "%.2f rad/s" % sensor_data[2] + "\r\n")

                        print(u'角度：')
                        print(u'\t x轴角度：' + "%.2f °" % rpy[0])
                        print(u'\t y轴角度：' + "%.2f °" % (- rpy[1]))
                        print(u'\t z轴角度：' + "%.2f °" % (- rpy[2] + 180) + "\r\n")

                        print(u'磁场：')
                        print(u'\t x轴磁场：' + "%.0f mG" % (sensor_data[6] * 1000))
                        print(u'\t y轴磁场：' + "%.0f mG" % (sensor_data[7] * 1000))
                        print(u'\t z轴磁场：' + "%.0f mG" % (sensor_data[8] * 1000) + "\r\n")

                time.sleep(0.002)

