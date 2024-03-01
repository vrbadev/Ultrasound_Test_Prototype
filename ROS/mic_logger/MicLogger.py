#!/usr/bin/env python

import rospy
from mic_logger.msg import ADCPacketRaw

import serial
import numpy as np
import time


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
        self.adc_channel = rospy.get_param("~adc_channel")
        self.adc_freq = rospy.get_param("~adc_freq")
        self.adc_samples_per_packet = rospy.get_param("~adc_samples_per_packet")
        self.adc_delim_seq = rospy.get_param("~adc_delim_seq")
        self.gain_adjustment_period = rospy.get_param("~gain_adjustment_period")
        self.settings_print_period = rospy.get_param("~settings_print_period")

        self.total_packet_bytes = len(self.adc_delim_seq) + 7 + int((2 * 3 * self.adc_samples_per_packet) // 4) + 2

        self.pub = rospy.Publisher("/mic_logger_data", ADCPacketRaw, queue_size=1)
    
    def get_cmd_pga_gain(self):
        g_index = np.argmin(np.abs(PGA_GAINS - self.pga_gain))
        self.pga_gain = PGA_GAINS[g_index]
        g = "G%d\n" % g_index
        return g.encode("ascii")

    def get_cmd_pga_offset(self):
        o_index = np.argmin(np.abs(PGA_OFFSETS - abs(self.pga_offset)))
        self.pga_offset = (1 if self.pga_offset >= 0 else -1) * PGA_OFFSETS[o_index]
        o = "O" + ("+" if self.pga_offset >= 0 else "-") + "%d\n" % o_index
        return o.encode("ascii")
    
    def get_cmd_adc_channel(self):
        c_index = min(2, max(0, self.adc_channel))
        c = "C%d\n" % c_index
        return c.encode("ascii")
    
    def set_pga_gain(self, com):
        cmd_gain = self.get_cmd_pga_gain()
        com.write(cmd_gain)
        com.flush()
        rospy.loginfo("[MicLogger] Sent CMD to set PGA gain: " + str(cmd_gain))

    def set_pga_offset(self, com):
        cmd_offset = self.get_cmd_pga_offset()
        com.write(cmd_offset)
        com.flush()
        rospy.loginfo("[MicLogger] Sent CMD to set PGA offset: " + str(cmd_offset))

    def set_adc_channel(self, com):
        cmd_channel = self.get_cmd_adc_channel()
        com.write(cmd_channel)
        com.flush()
        rospy.loginfo("[MicLogger] Sent CMD to set ADC channel: " + str(cmd_channel))

    def run(self):
        rospy.loginfo("[MicLogger] Launch settings: ADC channel=%d, PGA gain=%.2f, PGA offset=%.1f" % (self.adc_channel, self.pga_gain, self.pga_offset))

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

        self.set_pga_gain(com)
        rate.sleep()
        self.set_pga_offset(com)
        rate.sleep()
        self.set_adc_channel(com)
        rate.sleep()

        last_print_time = time.time()
        last_adjust_time = time.time()
        recv_bytes_cnt = 0
        recv_samples_cnt = 0

        pga_gain_id = 0xFF
        pga_offset_id = 0xFF
        adc_channel_id = 0xFF
        packet_id = -1

        while not rospy.is_shutdown():
            data_raw = com.read(self.total_packet_bytes)
            data = np.frombuffer(data_raw, dtype=np.uint8)
            if len(data) > 0:
                seq_pos = search_sequence(data, self.adc_delim_seq)
                if len(seq_pos) == 0:
                    continue
                elif len(seq_pos) == len(self.adc_delim_seq) and seq_pos[0] > 0:
                    rospy.loginfo("[MicLogger] Lost %d bytes, aligning to delimeter sequence at pos %d" % (self.total_packet_bytes - seq_pos[0], seq_pos[0]))
                    packet_id = -1
                    data = np.frombuffer(com.read(int(seq_pos[0])), dtype=np.uint8)
                else:
                    pga_gain_id = data[5]
                    pga_offset_id = np.frombuffer(data_raw[6:7], dtype=np.int8)
                    adc_channel_id = data[7]
                    packet_id = np.frombuffer(data_raw[8:12], dtype=np.uint32)
                    data = data[len(self.adc_delim_seq)+7:-2]
                
                if packet_id >= 0 and pga_gain_id <= 9 and abs(pga_offset_id) <= 15 and adc_channel_id <= 2:
                    recv_bytes_cnt += len(data)
                    new_adc_data = np.zeros(self.adc_samples_per_packet, dtype=np.uint16)
                    j = 0
                    for i in range(0, len(data), 3):
                        if i+1 < len(data):
                            new_adc_data[j] = data[i] | ((data[i+1] & 0x0F) << 8)
                            j += 1
                        if i+2 < len(data):
                            new_adc_data[j] = ((data[i+1] & 0xF0) << 4) | data[i+2]
                            j += 1
                    
                    recv_samples_cnt += j

                    msg = ADCPacketRaw()
                    msg.ros_stamp = rospy.Time.now()
                    msg.packet_id = int(packet_id)
                    msg.pga_gain = float(PGA_GAINS[pga_gain_id])
                    msg.pga_offset = float((-1 if pga_offset_id < 0 else 1) * PGA_OFFSETS[abs(pga_offset_id)])
                    msg.adc_channel = adc_channel_id
                    msg.adc_data = new_adc_data[:j]
                    self.pub.publish(msg)

                    adc_data_mean = new_adc_data.mean()
                    adc_data_min = new_adc_data.min()
                    adc_data_max = new_adc_data.max()

                    if self.settings_print_period > 0 and time.time() >= last_print_time + self.settings_print_period:
                        diff_time = time.time() - last_print_time
                        rospy.loginfo("[MicLogger] [#%d,G=%.2f,O=%.1f,CH%d] Received %d bytes (%d samples) in last %.3f sec -> %d B/s (%d sps); mean: %.2f; std: %.2f, min: %d, max: %d" % (packet_id, msg.pga_gain, msg.pga_offset, adc_channel_id, recv_bytes_cnt, recv_samples_cnt, diff_time, recv_bytes_cnt / diff_time, recv_samples_cnt/diff_time, adc_data_mean, adc_data_mean.std(), adc_data_min, adc_data_max))
                        last_print_time = time.time()
                        recv_bytes_cnt = 0
                        recv_samples_cnt = 0

                    if (self.gain_adjustment_period > 0 and time.time() >= last_adjust_time + self.gain_adjustment_period) or self.gain_adjustment_period == 0.0:
                        sel_gain_id = np.argmin(np.abs(PGA_GAINS - self.pga_gain))
                        if sel_gain_id == pga_gain_id:
                            dist = max(abs(adc_data_mean - adc_data_min), abs(adc_data_mean - adc_data_max))
                            if dist > 1800:
                                new_gain_id = max(0, pga_gain_id - 1)
                                if pga_gain_id != new_gain_id:
                                    rospy.loginfo("[MicLogger] The ADC values are too high, decreasing gain from %.2f to %.2f" % (PGA_GAINS[pga_gain_id], PGA_GAINS[new_gain_id]))
                                    self.pga_gain = PGA_GAINS[new_gain_id]
                                    self.set_pga_gain(com)
                            elif dist < 200:
                                new_gain_id = min(8, pga_gain_id + 1)
                                if pga_gain_id != new_gain_id:
                                    rospy.loginfo("[MicLogger] The ADC values are too low, increasing gain from %.2f to %.2f" % (PGA_GAINS[pga_gain_id], PGA_GAINS[new_gain_id]))
                                    self.pga_gain = PGA_GAINS[new_gain_id]
                                    self.set_pga_gain(com)
                        last_adjust_time = time.time()
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

