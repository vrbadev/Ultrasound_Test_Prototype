#!/usr/bin/env python

import rospy
from mic_logger.msg import UInt16ArrayStamped

import serial
import numpy as np


PGA_GAINS = np.array([1, 10, 20, 30, 40, 60, 80, 120, 157, 0.25], dtype=float)
PGA_OFFSETS = np.array([0.0, 1.3, 2.5, 3.8, 4.9, 6.1, 7.3, 8.4, 10.6, 11.7, 12.7, 13.7, 14.7, 15.7, 16.7, 17.6], dtype=float)

def search_sequence(arr, seq):
    arr = np.array(arr) if isinstance(arr, list) else arr
    seq = np.array(seq) if isinstance(seq, list) else seq
    Na, Nseq = arr.size, seq.size
    r_seq = np.arange(Nseq)
    M = (arr[np.arange(Na-Nseq+1)[:,None] + r_seq] == seq).all(1)
    if M.any() > 0:
        return np.where(np.convolve(M,np.ones((Nseq),dtype=int))>0)[0]
    else:
        return []
    

class MicLogger():
    def __init__(self):
        self.node = rospy.init_node("mic_logger")

        self.com_port = rospy.get_param("~com_port")
        self.com_baud = int(rospy.get_param("~com_baud"))
        self.com_cts = rospy.get_param("~com_cts")
        self.pga_gain = rospy.get_param("~pga_gain")
        self.pga_offset = rospy.get_param("~pga_offset")
        self.adc_freq = rospy.get_param("~adc_freq")
        self.adc_samples_per_packet = rospy.get_param("~adc_samples_per_packet")
        self.adc_delim_seq = rospy.get_param("~adc_delim_seq")

        self.total_packet_bytes = int((2 * 3 * self.adc_samples_per_packet) // 4) + len(self.adc_delim_seq)

        self.pub = rospy.Publisher("/mic_logger_data", UInt16ArrayStamped, queue_size=1)
    
    def pga_settings(self):
        g_index = np.argmin(np.abs(PGA_GAINS - self.pga_gain))
        o_index = np.argmin(np.abs(PGA_OFFSETS - abs(self.pga_offset)))
        
        self.pga_gain = PGA_GAINS[g_index]
        self.pga_offset = (1 if self.pga_offset >= 0 else -1) * PGA_OFFSETS[o_index]

        g = "G%d\n" % g_index
        o = "O" + ("+" if self.pga_offset >= 0 else "-") + "%d\n" % o_index
        return g.encode("ascii"), o.encode("ascii")

    def run(self):
        rospy.loginfo("[MicLogger] Opening COM: port=%s, baud=%d" % (self.com_port, self.com_baud))

        open_com = lambda: serial.Serial(self.com_port, baudrate=self.com_baud, timeout=1, dsrdtr=self.com_cts)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                com = open_com()
                break
            except Exception as e:
                rospy.logwarn("[MicLogger] Failed to open COM port: " + str(e))
                rate.sleep()

        rospy.loginfo("[MicLogger] COM port opened.")

        cmd_gain, cmd_offset = self.pga_settings()
        rospy.loginfo("[MicLogger] PGA settings: gain=%.2f, offset=%.1f" % (self.pga_gain, self.pga_offset))
        com.write(cmd_gain)
        com.flush()
        rospy.loginfo("[MicLogger] Sent CMD to set gain: " + str(cmd_gain))
        rate.sleep()
        com.write(cmd_offset)
        com.flush()
        rospy.loginfo("[MicLogger] Sent CMD to set offset: " + str(cmd_offset))
        rate.sleep()

        while not rospy.is_shutdown():
            data = np.frombuffer(com.read(self.total_packet_bytes), dtype=np.uint8)
            if len(data) > 0:
                seq_pos = search_sequence(data, self.adc_delim_seq)
                if len(seq_pos) == len(self.adc_delim_seq) and seq_pos[0] > 0:
                    rospy.loginfo("[MicLogger] Lost %d bytes, aligning to delimeter sequence at pos %d" % (self.total_packet_bytes - seq_pos[0], seq_pos[0]))
                    data = np.frombuffer(com.read(int(seq_pos[0])), dtype=np.uint8)
                else:
                    data = data[len(self.adc_delim_seq):]
                    
                new_adc_data = np.zeros(self.adc_samples_per_packet, dtype=np.uint16)
                j = 0
                for i in range(0, len(data), 3):
                    if i+1 < len(data):
                        new_adc_data[j] = data[i] | ((data[i+1] & 0x0F) << 8)
                        j += 1
                    if i+2 < len(data):
                        new_adc_data[j] = ((data[i+1] & 0xF0) << 4) | data[i+2]
                        j += 1
                
                msg = UInt16ArrayStamped()
                msg.header.stamp = rospy.Time.now()
                msg.data = new_adc_data[:j]
                self.pub.publish(msg)
            else:
                rospy.logwarn("[MicLogger] No data received, reopening COM port...")
                com.close()
                rate.sleep()
                com = open_com()
        
        com.close()
        rospy.loginfo("[MicLogger] COM port closed.")


if __name__ == '__main__':
    logger = MicLogger()
    logger.run()
    rospy.spin()

