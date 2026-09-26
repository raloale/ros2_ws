#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import numpy as np
import serial
import sys

from datetime import datetime
import struct

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.msg as sensor_msgs

class URadIndDriverNode(Node):

    def __init__(self):
        super().__init__("uRADindDriver")
        self.get_logger().info("Init")

        self.publisher_ = self.create_publisher(PointCloud2, "urad_topic", 1)

        self.configPort_name = '/dev/ttyUSB0'
        self.get_logger().info("\t+ configPort: %s" % self.configPort_name)
        self.dataPort_name = '/dev/ttyUSB1'
        self.get_logger().info("\t+ configPort: %s" % self.dataPort_name)
        self.usbConnector_upward = False     
        self.get_logger().info("\t+ usbConnector_upward: %s" % self.usbConnector_upward)
        self.pitch_angle = 0 * np.pi/180             
        self.get_logger().info("\t+ pitch_angle: %s" % self.pitch_angle)
        self.yaw_angle = 0 * np.pi/180    
        self.get_logger().info("\t+ yaw_angle: %s" % self.yaw_angle)
        self.channel =  1  
        if (self.channel == 2):
            freq_start = 60.75
        elif (self.channel == 3):
            freq_start = 61.5
        elif (self.channel == 4):
            freq_start = 62.25
        elif (self.channel == 5):
            freq_start = 63.00
        else:
            freq_start = 60.00
        self.get_logger().info("\t+ channel: %s --> %s GHz" % (self.channel, freq_start))

        fs = 20

        self.commands = [
                'sensorStop\n',
                'flushCfg\n',
                'dfeDataOutputMode 1\n',
                'channelCfg 15 7 0\n',
                'adcCfg 2 1\n',
                'adcbufCfg -1 0 1 1 1\n',
                'profileCfg 0 %1.2f 8 7 19.52 0 0 30 1 144 12499 0 0 158\n' % freq_start,
                'chirpCfg 0 0 0 0 0 0 0 1\n',
                'frameCfg 0 0 64 0 %d 1 0\n' % (np.round(1e3/fs)),
                'lowPower 0 0\n',
                'guiMonitor -1 1 0 0 0 0 0\n',
                'cfarCfg -1 0 2 8 4 3 0 10 1\n',
                'cfarCfg -1 1 0 8 4 4 1 10 1\n',
                'multiObjBeamForming -1 1 0.5\n',
                'clutterRemoval -1 1\n',
                'calibDcRangeSig -1 0 -5 8 256\n',
                'extendedMaxVelocity -1 1\n',
                'lvdsStreamCfg -1 0 0 0\n',
                'compRangeBiasAndRxChanPhase 0.0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0\n',
                'measureRangeBiasAndRxChanPhase 0 1.5 0.2\n',
                'CQRxSatMonitor 0 3 4 35 0\n',
                'CQSigImgMonitor 0 71 4\n',
                'analogMonitor 0 0\n',
                'aoaFovCfg -1 -90 90 -90 90\n',
                'cfarFovCfg -1 0 0 49.99\n',
                'cfarFovCfg -1 1 -45.42 45.42\n',
                'calibData 0 0 0\n',
                'sensorStart\n'
            ]

        self.get_logger().info("Started")

    def rotatePoints(self, x, y, z):

        x_ = np.zeros(len(x))
        y_ = np.zeros(len(y))
        z_ = np.zeros(len(z))

        Rx = np.matrix([[1,0,0],[0,np.cos(self.pitch_angle), -np.sin(self.pitch_angle)],[0, np.sin(self.pitch_angle), np.cos(self.pitch_angle)]])
        Rz = np.matrix([[np.cos(self.yaw_angle), -np.sin(self.yaw_angle), 0], [np.sin(self.yaw_angle), np.cos(self.yaw_angle), 0], [0,0,1]])

        R = np.matmul(Rz, Rx)

        for i in range(len(x)):
            outputPoint = np.matmul(R, np.array([x[i],y[i],z[i]]))
            x_[i] = outputPoint[0,0]
            y_[i] = outputPoint[0,1]
            z_[i] = outputPoint[0,2]

        return x_, y_, z_

    def initializeRadar(self):
        self.configPort = serial.Serial(self.configPort_name, 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.3)
        self.dataPort = serial.Serial(self.dataPort_name, 921600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.5)
        self.dataPort.reset_output_buffer()

        response = bytearray([])
        while(self.configPort.in_waiting > 0):
            response += self.configPort.read(1)
        self.get_logger().info('initializeRadar [A]: %s' % (response.decode()) )

        for i in range(len(self.commands)):
            #logging.info('[%s] initializeRadar [C.%s]: %s' % (self.serviceId, i, self.commands[i]) )
            self.configPort.write(bytearray(self.commands[i].encode()))
            time.sleep(20e-3)
            response = bytearray([])
            while(self.configPort.in_waiting > 0):
                response += self.configPort.read(1)
            self.get_logger().info('initializeRadar [D]: %s' % (response.decode()) )

    def releaseRadar(self):
        self.get_logger().info('releaseRadar')
        if self.configPort.is_open:
            self.configPort.write(bytearray('sensorStop\n'.encode()))
            time.sleep(20e-3)
            response = bytearray([])
            while(self.configPort.in_waiting > 0):
                response += self.configPort.read(1)
            self.get_logger().info('releaseRadar [D]: %s' % (response.decode()) )
            self.configPort.close()
        if self.dataPort.is_open:
           self.dataPort.close()

    def run(self):
        self.get_logger().info('run')
        t = 0
        ts = time.time()

        self.connected = False
        slot = 0

        packetHeader = bytearray([])

        syncPattern = 0x708050603040102
        offset_cpu_time = 0
        overflows_cpu_time = 0
        frequencySysClock = 200e6
        marginCpuCycles = frequencySysClock * 1
        timeCpuCycles_prev = 0

        tlvHeaderLen = 8
        headerLen = 40

        defined_timePacket_0 = False
        timePacket_0 = time.time()

        while rclpy.ok():
            if not(self.connected):
                try:
                    self.initializeRadar()
                    self.connected = True

                except Exception as e:
                    self.releaseRadar()
                    self.get_logger().error('Error %s' % (sys.exc_info()[0]))
                    self.get_logger().error(e, exc_info=True)
                    time.sleep(10)
            else:
                try:
                    slot = slot + 1
                    if slot > sys.maxsize:
                        slot = 0

                    ts = time.time()
                    datetimePacket = datetime.now()
                    packetHeader += self.dataPort.read(headerLen-len(packetHeader))
                    reset_and_initialize = False
                    corruptedPacket = False

                    if (len(packetHeader) == headerLen):
                        sync, version, totalPacketLen, platform, frameNumber, timeCpuCycles, numDetectedObj, numTLVs, subFrameNumber =  struct.unpack('Q8I', packetHeader[:headerLen])

                        if (sync == syncPattern):
                            if (not defined_timePacket_0 and frameNumber >= 2):
                                timePacket_0 = ts
                                offset_cpu_time = timeCpuCycles/frequencySysClock - 0.1
                                defined_timePacket_0 = True

                            #if (saveRawData):
                            #    fileStats = open('./%s/%04d-%02d-%02d_%s' % (foldername, datetimePacket.year, datetimePacket.month, datetimePacket.day, stats_filename), 'a')
                            #    fileStats.write('%d %d %1.3f\n' % (timeCpuCycles, frameNumber, ts))
                            #    fileStats.close()
                            
                            if (abs(timeCpuCycles-timeCpuCycles_prev) > (2**32 - marginCpuCycles)):
                                overflows_cpu_time += 1
                            timeCpuCycles_prev = timeCpuCycles
                            timestampCpuCycles = timeCpuCycles + overflows_cpu_time*(2**32)
                            timestampCpuCycles /= frequencySysClock

                            ti = timePacket_0 + timestampCpuCycles - offset_cpu_time

                            packetHeader = bytearray([])
                            if (numDetectedObj <= 400 and totalPacketLen < 12000 and numTLVs < 10):
                                packetPayload = self.dataPort.read(totalPacketLen-headerLen)

                                if (len(packetPayload) == totalPacketLen-headerLen):

                                    detectedObjects = np.zeros((numDetectedObj, 6))
                                    #if (printPointCloud):
                                    #    print('numDetectedObj: %d' % numDetectedObj)

                                    for i in range(numTLVs):
                                        tlvType, tlvLength = struct.unpack('2I', packetPayload[:tlvHeaderLen])

                                        if (tlvType > 20 or tlvLength > 10000):
                                            packetHeader = bytearray([])
                                            corruptedPacket = True
                                            break

                                        packetPayload = packetPayload[tlvHeaderLen:]

                                        if (tlvType == 1):

                                            for j in range(numDetectedObj):

                                                x, y, z, v = struct.unpack('4f', packetPayload[:16])
                                                
                                                detectedObjects[j, 0] = x
                                                detectedObjects[j, 1] = y
                                                detectedObjects[j, 2] = z
                                                detectedObjects[j, 3] = v

                                                packetPayload = packetPayload[16:]

                                        elif (tlvType == 7):

                                            for j in range(numDetectedObj):

                                                snr, noise = struct.unpack('2H', packetPayload[:4])

                                                detectedObjects[j, 4] = snr
                                                detectedObjects[j, 5] = noise

                                                #if (printPointCloud):
                                                #self.get_logger().info('x: %1.3f m, y: %1.3f m, z: %1.3f m, v: %1.3f m/s, snr: %d, noise: %d' % (detectedObjects[j, 0], detectedObjects[j, 1], detectedObjects[j, 2], detectedObjects[j, 3], detectedObjects[j, 4], detectedObjects[j, 5]))

                                                packetPayload = packetPayload[4:]

                                    if (not corruptedPacket):
                                        x = detectedObjects[:numDetectedObj, 0]
                                        y = detectedObjects[:numDetectedObj, 1]
                                        z = detectedObjects[:numDetectedObj, 2]

                                        if (not self.usbConnector_upward):
                                            x = -x
                                            z = -z

                                        x, y, z = self.rotatePoints(x, y, z)
                                        Range = np.sqrt(x**2+y**2+z**2)
                                        Azimuth = np.arctan(x/y)
                                        Elevation = np.arccos(z/Range)
                                        Velocity = detectedObjects[:numDetectedObj, 3]
                                        Amplitude = (detectedObjects[:numDetectedObj, 4] + detectedObjects[:numDetectedObj, 5])/10
                                        SNR = detectedObjects[:numDetectedObj, 4]/10

                                        for j in range(numDetectedObj):
                                            pass
                                            #self.datalogger.info('%s; PLOT3D; ts=%s; slot=%s; index=%s; x=%1.3f; y=%1.3f; z=%1.3f; speed=%1.3f; amplitude=%1.3f; snr=%1.3f;' % (self.serviceId, ts, slot, j, x[j], y[j], z[j], Velocity[j], Amplitude[j], SNR[j]))

                                        #points = np.array([[1, 1, 1], [0, 0, 0]])
                                        #points = np.array(detectedObjects[:,0:3])
                                        points = np.array(detectedObjects)
                                        #print(points.shape, obj.shape)
                                        
                                        ros_dtype = sensor_msgs.PointField.FLOAT32
                                        dtype = np.float32
                                        itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

                                        data = points.astype(dtype).tobytes()
                                        #fields = [sensor_msgs.PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
                                        #        for i, n in enumerate('xyz')]
                                        fields = [sensor_msgs.PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
                                                for i, n in enumerate(['x','y','z','speed','snr','noise'])]

                                        header = std_msgs.msg.Header(frame_id='laser_link')

                                        pc2 = PointCloud2(
                                            header=header,
                                            height=1,
                                            width=points.shape[0],
                                            is_dense=False,
                                            is_bigendian=False,
                                            fields=fields,
                                            #point_step=(itemsize * 3),  # Every point consists of three float32s.
                                            point_step=(itemsize * 6),  # Every point consists of three float32s.
                                            #row_step=(itemsize * 3 * points.shape[0]),
                                            row_step=(itemsize * 6 * points.shape[0]),
                                            data=data
                                        )

                                        self.publisher_.publish(pc2)

                                        #_features_filename = './%s/%04d-%02d-%02d_%s' % (foldername, datetimePacket.year, datetimePacket.month, datetimePacket.day, features_filename)
                                        #_output_filename = './%s/%04d-%02d-%02d_%s' % (foldername, datetimePacket.year, datetimePacket.month, datetimePacket.day, output_filename)
                                        #condition = np.logical_and.reduce((x >= x_min, x <= x_max))
                                        #numberOfTrackings, Tracking, PointCloud, numberOfVehicles, Vehicles, numberOfVehicles_quasiDef, Vehicles_quasiDef, numberOfVehicles_def, Vehicles_def = uRAD_Tracking.track_Vehicles(True, numberOfTrackings, Tracking, PointCloud, numberOfVehicles, Vehicles, numberOfVehicles_quasiDef, Vehicles_quasiDef, ti, Range[condition], Velocity[condition], Azimuth[condition], Elevation[condition], Amplitude[condition], SNR[condition], _features_filename, _output_filename)

                                        #for j in range(numberOfVehicles_def):
                                        #    print('New vehicle, type: %d' % Vehicles_def[j].vehicle_Type)

                                        #if (printPointCloud and numTLVs > 0):
                                        #    print(' ')
                                else:
                                    reset_and_initialize = True
                        else:
                            packetHeader = packetHeader[1:]
                    else:
                        reset_and_initialize = True
                    
                    if (reset_and_initialize):
                        self.connected = False

                        packetHeader = bytearray([])

                        defined_timePacket_0 = False
                        timePacket_0 = time()


                except Exception as e:
                    self.connected = False
                    self.releaseRadar()
                    self.get_logger().error('[run] Error on connect dutty. Cause: %s' % (sys.exc_info()[0]))
                    self.get_logger().error(e, exc_info=True)
        
        if self.connected:
            self.releaseRadar()

         

def main(args=None):
    rclpy.init(args=args)
    node = URadIndDriverNode()

    #node.initializeRadar()

    node.run()

    #node.releaseRadar()

    rclpy.shutdown()

if __name__ == "__main__":
    main()