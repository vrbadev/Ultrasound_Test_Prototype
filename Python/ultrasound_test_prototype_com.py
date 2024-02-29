# -*- coding: utf-8 -*-
"""
Created on Sat Feb 10 13:34:15 2024
@author: Vojtech Vrba (vrbavoj3@fel.cvut.cz)
"""

import serial
import signal
import time
import matplotlib.pyplot as plt
import numpy as np

import _pickle as pickle
import multiprocessing
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg



ADC_SAMPLE_FREQ_HZ = 160e3
ADC_SAMPLES_PER_PACKET = 500
DELIM_SEQ = [0xFF] * 5

PACKET_TOTAL_LEN = len(DELIM_SEQ) + 7 + ((2 * ADC_SAMPLES_PER_PACKET * 3) // 4) + 2
PACKET_PERIOD_SEC = ADC_SAMPLE_FREQ_HZ / ADC_SAMPLES_PER_PACKET

COM_PORT = "COM5"
COM_BAUDRATE = 4000000
COM_CTS_USED = True

FFT_CALC_LEN = 2**15
FFT_CALC_PERIOD_SEC = 0.1
FFT_VIS_UNDERSAMPLING = 64

SIGNAL_VIS_LEN_SEC = 5 


def tksleep(self, time:float) -> None:
    self.after(int(time*1000), self.quit)
    self.mainloop()
tk.Misc.tksleep = tksleep
        
class Visualizer(multiprocessing.Process):
    def __init__(self, title, size="1600x600", fps=30):
        super(Visualizer, self).__init__()
        self.term_evt = multiprocessing.Event()
        self.signal_queue = multiprocessing.Queue()
        self.fft_queue = multiprocessing.Queue()
        self.title = title
        self.size = size
        self.fps = fps
        
    def run(self):
        root = tk.Tk()
        root.title(self.title)
        root.geometry(self.size)
        root.protocol("WM_DELETE_WINDOW", self.close)
        
        fig = plt.figure(figsize=(16, 6), dpi=150)
        ax1 = fig.add_subplot(121)
        ax2 = fig.add_subplot(122)
        canvas = FigureCanvasTkAgg(fig, master=root)  
        canvas.draw() 
        canvas.get_tk_widget().pack()
        
        t_last = 0
        ys = list()
        ts = list()
        fft_fs = np.array([])
        fft_ys = np.array([])
        max_y_fft = 0
        
        update_signal = False
        update_fft = False
        
        while not self.is_terminated():
            while not self.signal_queue.empty():
                t_delta, y = self.signal_queue.get(False)
                t_last += t_delta
                ts.append(t_last)
                ys.append(y)
                update_signal = True
                
            while not self.fft_queue.empty():
                fft_ys = pickle.loads(self.fft_queue.get(False))
                fft_fs = np.linspace(0, ADC_SAMPLE_FREQ_HZ/2e3, len(fft_ys))
                max_y_fft = max(max_y_fft, fft_ys.max())
                update_fft = True
            
            if update_signal:
                ax1.clear()
                plot_len = min(len(ys), int(PACKET_PERIOD_SEC * SIGNAL_VIS_LEN_SEC))
                ax1.plot(ts[-plot_len:], ys[-plot_len:])
                ax1.grid()
                ax1.set_ylim([-0.1, 3.5])
                ax1.set_xlabel("Time [s]")
                ax1.set_ylabel("Amplitude [V]")
                update_signal = False
                
            if update_fft:
                ax2.clear()
                ax2.plot(fft_fs, fft_ys)
                ax2.grid()
                ax2.set_ylim([-0.1, max_y_fft])
                ax2.set_xlabel("Frequency [kHz]")
                ax2.set_ylabel("Amplitude")
                update_fft = False
                
            canvas.draw()
            root.update()
            
            root.tksleep(1.0 / self.fps)
        
        root.quit()
    
    def close(self):
        self.term_evt.set()
        
    def is_terminated(self):
        return self.term_evt.is_set()

class Control(multiprocessing.Process):
    GAINS = [1, 10, 20, 30, 40, 60, 80, 120, 157, 0.25]
    OFFSETS = [0.0, 1.3, 2.5, 3.8, 4.9, 6.1, 7.3, 8.4, 10.6, 11.7, 12.7, 13.7, 14.7, 15.7, 16.7, 17.6]
    CHANNELS = [0, 1, 2]
    
    def __init__(self, size="250x200"):
        super(Control, self).__init__()
        self.term_evt = multiprocessing.Event()
        self.ctrl_queue = multiprocessing.Queue()
        self.size = size
        
    def run(self):
        root = tk.Tk()
        root.title("Controls")
        root.geometry(self.size)
        root.protocol("WM_DELETE_WINDOW", self.close)
        
        frame = tk.Frame(root)
        
        gains_str = {("%.2fx" % gain): ("G%d" % i) for i, gain in enumerate(Control.GAINS)}
        offsets_str = {("-%.1fmV" % offset): ("O-%d" % (len(Control.OFFSETS) - i - 1)) for i, offset in enumerate(Control.OFFSETS[::-1])} 
        offsets_str |= {("+%.1fmV" % offset): ("O+%d" % i) for i, offset in enumerate(Control.OFFSETS)}
        channels_str = {("CH%d" % channel): ("C%d" % i) for i, channel in enumerate(Control.CHANNELS)}
        
        selected_gain_str = tk.StringVar(root)
        selected_gain_str.set("157.00x")
        selected_offset_str = tk.StringVar(root)
        selected_offset_str.set("+10.6mV")
        selected_channel_str = tk.StringVar(root)
        selected_channel_str.set("CH1")
        
        tk.Label(frame, text="PGA gain:").grid(row=0, column=0, sticky=tk.E)
        tk.OptionMenu(frame, selected_gain_str, *list(gains_str.keys())).grid(row=0, column=1, sticky=tk.W)
        tk.Button(frame, text="Set", command=lambda: self.ctrl_queue.put(gains_str[selected_gain_str.get()])).grid(row=0, column=2, sticky=tk.W)

        tk.Label(frame, text="PGA offset:").grid(row=1, column=0, sticky=tk.E)
        tk.OptionMenu(frame, selected_offset_str, *list(offsets_str.keys())).grid(row=1, column=1, sticky=tk.W)
        tk.Button(frame, text="Set", command=lambda: self.ctrl_queue.put(offsets_str[selected_offset_str.get()])).grid(row=1, column=2, sticky=tk.W)
        
        tk.Label(frame, text="ADC channel:").grid(row=2, column=0, sticky=tk.E)
        tk.OptionMenu(frame, selected_channel_str, *list(channels_str.keys())).grid(row=2, column=1, sticky=tk.W)
        tk.Button(frame, text="Set", command=lambda: self.ctrl_queue.put(channels_str[selected_channel_str.get()])).grid(row=2, column=2, sticky=tk.W)
        
        frame.pack(expand=True)
        
        while not self.is_terminated():
            root.update()
            
        root.quit()
    
    def close(self):
        self.term_evt.set()
        
    def is_terminated(self):
        return self.term_evt.is_set()

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


if __name__ == "__main__":
    terminated = False
    signal.signal(signal.SIGINT, lambda n, f: globals().update(terminated=True))
    
    com = serial.Serial(COM_PORT, baudrate=COM_BAUDRATE, timeout=1)
    print("COM port opened!")
    
    adc_data = list()

    last_print_time = time.time()
    last_fft_time = time.time()
    recv_bytes_cnt = 0
    recv_samples_cnt = 0
    
    window_vis = Visualizer("Signal & FFT Spectrum")
    window_vis.start()
    
    window_ctrl = Control()
    window_ctrl.start()
    
    print("Waiting for delimeter for initial alignment...")
    
    while not terminated and not window_vis.is_terminated() and not window_ctrl.is_terminated():
        data_raw = com.read(PACKET_TOTAL_LEN)
        data = np.frombuffer(data_raw, dtype=np.uint8)
        seq_pos = search_sequence(data, DELIM_SEQ)
        if len(seq_pos) == 0:
            continue
        elif len(seq_pos) == len(DELIM_SEQ) and seq_pos[0] > 0:
            print("Lost %d bytes, aligning to delimeter sequence at pos %d" % (PACKET_TOTAL_LEN - seq_pos[0], seq_pos[0]))
            com.read(int(seq_pos[0])) # dummy read
        else:           
            pga_gain_id = data[5]
            pga_offset_id = np.frombuffer(data_raw[6:7], dtype=np.int8)
            adc_channel_id = data[7]
            packet_id = np.frombuffer(data_raw[8:12], dtype=np.uint32)
            
            data = data[len(DELIM_SEQ)+7:-2] 
            recv_bytes_cnt += len(data)
            
            new_adc_data = np.zeros(ADC_SAMPLES_PER_PACKET, dtype=np.uint16)
            j = 0
            for i in range(0, len(data), 3):
                new_adc_data[j] = data[i] | ((data[i+1] & 0x0F) << 8)
                new_adc_data[j+1] = ((data[i+1] & 0xF0) << 4) | data[i+2]
                j += 2
                recv_samples_cnt += 2
            
            adc_data.extend(new_adc_data.tolist())
            window_vis.signal_queue.put((ADC_SAMPLES_PER_PACKET / ADC_SAMPLE_FREQ_HZ, new_adc_data.mean() * 3.3 / 4096))
               
            if time.time() >= last_fft_time + FFT_CALC_PERIOD_SEC:
                if len(adc_data) >= FFT_CALC_LEN:
                    arr = np.array(adc_data[-FFT_CALC_LEN:], dtype=float) * 3.3 / 4096
                    arr -= arr.mean()
                    spectrum = np.abs(np.fft.fft(arr))[:(FFT_CALC_LEN // 2)]
                    spectrum_undersampled = np.hstack([spectrum[i:i+FFT_VIS_UNDERSAMPLING].max() for i in range(0, len(spectrum), FFT_VIS_UNDERSAMPLING)])
                    window_vis.fft_queue.put(pickle.dumps(spectrum_undersampled, protocol=1))
                last_fft_time = time.time()
            
            if time.time() >= last_print_time + 1:
                diff_time = time.time() - last_print_time
                print("[#%d,G%d,O%d,C%d] Received %d bytes (%d samples) in last %.3f sec -> %d B/s (%d sps); mean: %.2f; std: %.2f" % (packet_id, pga_gain_id,pga_offset_id, adc_channel_id, recv_bytes_cnt, recv_samples_cnt, diff_time, recv_bytes_cnt / diff_time, recv_samples_cnt/diff_time, np.mean(new_adc_data), np.std(new_adc_data)))
                last_print_time = time.time()
                recv_bytes_cnt = 0
                recv_samples_cnt = 0
        
        while not window_ctrl.ctrl_queue.empty():
            cmd = window_ctrl.ctrl_queue.get()
            print("Sending command: " + cmd)
            com.write((cmd + "\n").encode("ascii"))
            
        
    com.close()
    window_vis.close()
    window_ctrl.close()
    
    print("Program End.")
