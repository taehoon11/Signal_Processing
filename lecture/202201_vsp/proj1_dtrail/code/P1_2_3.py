import numpy as np
import os
import glob
import csv
import matplotlib
from matplotlib import cm
from matplotlib import pyplot as plt                


data_path = '/workspace/202201_vsp/proj1_dtrail/db_rosbag'  # 데이터 경로

def get_filelist(path, ext):                              # 데이터를 불러오기 위한 함수1
    filelist = [f for f in glob.glob(path + "*." + ext)]
    return filelist


def get_ra(ra_file):                                      # 데이터를 불러오기 위한 함수2
    cnt = 0
    rows_num = 0
    cols_num = 0
    with open(ra_file) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            #print(len(row))
            cols_num = len(row)
            rows_num += 1
    ra_np = np.zeros((rows_num, cols_num))
    #print('File:%s, (%d,%d)'%(ra_file, rows_num, cols_num))

    with open(ra_file) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            ra_np[cnt,:] = row
            cnt += 1
    return ra_np



def Do_FFT(signal_list):                          # FFT(Fast Fourier Transform)을 위한 함수
    FFT_OUT_PUT = np.fft.fft(signal_list)
    amp = abs(FFT_OUT_PUT) 
    
    return FFT_OUT_PUT,amp

def Do_IFFT(signal_list):                        # IFFT를 위한 함수
    IFFT_OUT_PUT = np.fft.ifft(signal_list)

    return IFFT_OUT_PUT  


def freq_idx_line(any_dataset,FT_freq):          # LPF, HPF, BPF를 위한 시간-주파수 관계 (필터제작-1) 함수
    L = len(any_dataset)
    m = L//2
    slope = FT_freq[m]/m
    return slope


def Change_Target_Hz2idx(freq_slope,hz):      # LPF, HPF, BPF를 위한 시간-주파수 관계 (필터제작-2) in=desire_Hz, out=matched_idx
    idx = hz/freq_slope                       # 없애길 원하는 Hz를 넣으면 해당 index반환 필터들에서 이것을 기반으로 연산
    return idx


def Do_LOW_PASS_Filter(F_domain_signal,Threshold): # Kill Threshold_Hz < freq  data (필터제작-3)
    for i in range(len(F_domain_signal)):
        if i < Threshold or i > len(F_domain_signal) - Threshold:
            F_domain_signal[i] = F_domain_signal[i]
        else:
            F_domain_signal[i] = 0
    return F_domain_signal


def Do_High_PASS_Filter(F_domain_signal,Threshold): # Kill Threshold_Hz < freq  data (필터제작-4)
    for i in range(len(F_domain_signal)):
        if i < Threshold or i > len(F_domain_signal) - Threshold:
            F_domain_signal[i] = 0
        else:
            F_domain_signal[i] = F_domain_signal[i]
    return F_domain_signal

def Do_BEND_PASS_Filter(F_domain_signal,Threshold1,Threshold2):   #Threshold1 > Threshold2 (필터제작-5)
    a = F_domain_signal[0]
    b = F_domain_signal[-1]
    for i in range(len(F_domain_signal)):
        
        if i > Threshold1 and i < len(F_domain_signal) - Threshold1:
            F_domain_signal[i] = 0
        elif i < Threshold2 or i > len(F_domain_signal) - Threshold2:
            F_domain_signal[i] = 0
        else:
            F_domain_signal[i] = F_domain_signal[i]

    return F_domain_signal



def Problem1():     # 1번 문항 : function for 3ang_vel & 3lin_acc plot

    plt.figure(figsize=(12,8))
    plt.subplot(3, 2, 1) 
    plt.plot(time,imu_ang_vel_raw_x,'r',label = 'ang_vel_roll')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('Roll Rate[rad/s]')
    plt.title('ang_vel_raw[Time-Domain]')
    plt.grid()        

    plt.subplot(3, 2, 3)
    plt.plot(time,imu_ang_vel_raw_y,'g',label = 'ang_vel_pitch')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('Pitch Rate[rad/s]')
    plt.grid()
        
    plt.subplot(3, 2, 5)
    plt.plot(time,imu_ang_vel_raw_z,'b',label = 'ang_vel_yaw')
    plt.legend(loc=(0.67, 0.8))
    plt.xlabel('Time(Sec)')
    plt.ylabel('Yaw Rate[rad/s]')
    plt.grid()

    plt.subplot(3, 2, 2) 
    plt.plot(time,imu_lin_acc_x,'r',label = 'lin_acc_x')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('f_x[m/s^2]')
    plt.title('lin_acc[Time-Domain]')        
    plt.grid()
       
    plt.subplot(3, 2, 4)
    plt.plot(time,imu_lin_acc_y,'g',label = 'lin_acc_y')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('f_y[m/s^2]')
    plt.grid()

    plt.subplot(3, 2, 6)
    plt.plot(time,imu_lin_acc_z,'b',label = 'lin_acc_z')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('f_z[m/s^2]')
    plt.xlabel('Time(Sec)')
    plt.grid()
        

    plt.show() 

def Problem2():   # 2번 문항 : function for show 6 Robot motion in Freq-Domain

    FD_acc_x,acc_amp_x = Do_FFT(imu_lin_acc_x)
    FD_acc_y,acc_amp_y = Do_FFT(imu_lin_acc_y)
    FD_acc_z,acc_amp_z = Do_FFT(imu_lin_acc_z)
    FD_ang_vel_x,ang_vel_amp_x = Do_FFT(imu_ang_vel_raw_x)
    FD_ang_vel_y,ang_vel_amp_y = Do_FFT(imu_ang_vel_raw_y)
    FD_ang_vel_z,ang_vel_amp_z = Do_FFT(imu_ang_vel_raw_z)

    plt.figure(figsize=(12,8))
    plt.subplot(3, 2, 1) 
    plt.plot(FT_freq,ang_vel_amp_x,'r',label = 'ang_vel_roll')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('amplitude')
    plt.title('ang_vel_raw[Freq-Domain]')
    plt.grid()        

    plt.subplot(3, 2, 3)
    plt.plot(FT_freq,ang_vel_amp_y,'g',label = 'ang_vel_pitch')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('amplitude')
    plt.grid()
        
    plt.subplot(3, 2, 5)
    plt.plot(FT_freq,ang_vel_amp_z,'b',label = 'ang_vel_yaw')
    plt.legend(loc=(0.67, 0.8))
    plt.xlabel('Frequency[Hz]')
    plt.ylabel('amplitude')
    plt.grid()

    plt.subplot(3, 2, 2) 
    plt.plot(FT_freq,acc_amp_x,'r',label = 'lin_acc_x')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('amplitude')
    plt.title('lin_acc[Freq-Domain]')      
    plt.grid()
       
    plt.subplot(3, 2, 4)
    plt.plot(FT_freq,acc_amp_y,'g',label = 'lin_acc_y')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('amplitude')
    plt.grid()

    plt.subplot(3, 2, 6)
    plt.plot(FT_freq,acc_amp_z,'b',label = 'lin_acc_z')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('amplitude')
    plt.xlabel('Frequency[Hz]')
    plt.grid()
        

    plt.show() 

def Problem3():        # 3번 문항 : function for compare Filtered(use LPF, BPF, HPF) & Original datain T-Domain 

    slope =freq_idx_line(imu_ang_vel_raw_x,FT_freq)

    FD_acc_x,acc_amp_x = Do_FFT(imu_lin_acc_x)
    FD_acc_y,acc_amp_y = Do_FFT(imu_lin_acc_y)
    FD_acc_z,acc_amp_z = Do_FFT(imu_lin_acc_z)
    FD_ang_vel_x,ang_vel_amp_x = Do_FFT(imu_ang_vel_raw_x)
    FD_ang_vel_y,ang_vel_amp_y = Do_FFT(imu_ang_vel_raw_y)
    FD_ang_vel_z,ang_vel_amp_z = Do_FFT(imu_ang_vel_raw_z)


    
    new_ang_vel_x = Do_LOW_PASS_Filter(FD_ang_vel_x,Change_Target_Hz2idx(slope,1.5)) # cut freq = 1.5Hz
    new_ang_vel_y = Do_BEND_PASS_Filter(FD_ang_vel_y,Change_Target_Hz2idx(slope,3),Change_Target_Hz2idx(slope,1)) # want to kill without= 1~3Hz
    new_ang_vel_z = Do_LOW_PASS_Filter(FD_ang_vel_z,Change_Target_Hz2idx(slope,1))  # cut freq = 1Hz
    new_acc_x = Do_LOW_PASS_Filter(FD_acc_x,Change_Target_Hz2idx(slope,3))   # cut freq = 3Hz
    new_acc_y = Do_BEND_PASS_Filter(FD_acc_y,Change_Target_Hz2idx(slope,10),Change_Target_Hz2idx(slope,4))  # want to kill without= 4~10Hz
    new_acc_z = Do_BEND_PASS_Filter(FD_acc_z,Change_Target_Hz2idx(slope,22),Change_Target_Hz2idx(slope,15)) # want to kill without= 15~22Hz

    new1_ang_vel_x = Do_IFFT(new_ang_vel_x)
    new1_ang_vel_y = Do_IFFT(new_ang_vel_y)
    new1_ang_vel_z = Do_IFFT(new_ang_vel_z)

    new1_acc_x = Do_IFFT(new_acc_x)
    new1_acc_y = Do_IFFT(new_acc_y)
    new1_acc_z = Do_IFFT(new_acc_z)


    plt.figure(figsize=(16,12))
    plt.subplot(3, 2, 1) 
    plt.plot(time,imu_ang_vel_raw_x,'y',label = 'ang_vel_roll')
    plt.plot(time,new1_ang_vel_x,'r',label = 'ang_vel_roll_filtered')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('Roll Rate[rad/s]')
    plt.title('ang_vel_raw[Time-Domain]')
    plt.grid()        

    plt.subplot(3, 2, 3)
    plt.plot(time,imu_ang_vel_raw_y,'y',label = 'ang_vel_pitch')
    plt.plot(time,new1_ang_vel_y,'g',label = 'ang_vel_pitch_filtered')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('Pitch Rate[rad/s]')
    plt.grid()
        
    plt.subplot(3, 2, 5)
    plt.plot(time,imu_ang_vel_raw_z,'y',label = 'ang_vel_yaw')
    plt.plot(time,new1_ang_vel_z,'b',label = 'ang_vel_yaw_filtered')
    plt.legend(loc=(0.67, 0.8))
    plt.xlabel('Time(Sec)')
    plt.ylabel('Yaw Rate[rad/s]')
    plt.grid()

    plt.subplot(3, 2, 2)
    plt.plot(time,imu_lin_acc_x,'y',label = 'lin_acc_x') 
    plt.plot(time,new1_acc_x,'r',label = 'lin_acc_x_filtered')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('f_x[m/s^2]')
    plt.title('lin_acc[Time_domain]')        
    plt.grid()
       
    plt.subplot(3, 2, 4)
    plt.plot(time,imu_lin_acc_y,'y',label = 'lin_acc_y')
    plt.plot(time,new1_acc_y,'g',label = 'lin_acc_y_filtered')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('f_y[m/s^2]')
    plt.grid()

    plt.subplot(3, 2, 6)
    plt.plot(time,imu_lin_acc_z,'y',label = 'lin_acc_z')
    plt.plot(time,new1_acc_z,'b',label = 'lin_acc_z_filtered')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('f_z[m/s^2]')
    plt.xlabel('Time(Sec)')
    plt.grid()
        

    plt.show()  

def Problem3_0():                 # 추가적인 함수 : function for show 6 filtered data in Freq_Domain
    slope =freq_idx_line(imu_ang_vel_raw_x,FT_freq)

    FD_acc_x,acc_amp_x = Do_FFT(imu_lin_acc_x)
    FD_acc_y,acc_amp_y = Do_FFT(imu_lin_acc_y)
    FD_acc_z,acc_amp_z = Do_FFT(imu_lin_acc_z)
    FD_ang_vel_x,ang_vel_amp_x = Do_FFT(imu_ang_vel_raw_x)
    FD_ang_vel_y,ang_vel_amp_y = Do_FFT(imu_ang_vel_raw_y)
    FD_ang_vel_z,ang_vel_amp_z = Do_FFT(imu_ang_vel_raw_z)


    new_ang_vel_x = Do_LOW_PASS_Filter(FD_ang_vel_x,Change_Target_Hz2idx(slope,1.5))
    new_ang_vel_y = Do_BEND_PASS_Filter(FD_ang_vel_y,Change_Target_Hz2idx(slope,3),Change_Target_Hz2idx(slope,1))    
    new_ang_vel_z = Do_LOW_PASS_Filter(FD_ang_vel_z,Change_Target_Hz2idx(slope,1))
    new_acc_x = Do_LOW_PASS_Filter(FD_acc_x,Change_Target_Hz2idx(slope,3))
    new_acc_y = Do_BEND_PASS_Filter(FD_acc_y,Change_Target_Hz2idx(slope,10),Change_Target_Hz2idx(slope,4))
    new_acc_z = Do_BEND_PASS_Filter(FD_acc_z,Change_Target_Hz2idx(slope,22),Change_Target_Hz2idx(slope,15))


    plt.figure(figsize=(12,8))
    plt.subplot(3, 2, 1) 
    plt.plot(FT_freq,abs(new_ang_vel_x),'r',label = 'ang_vel_roll')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('amplitude')
    plt.title('filtered_ang_vel[Freq-Domain]')
    plt.grid()        

    plt.subplot(3, 2, 3)
    plt.plot(FT_freq,abs(new_ang_vel_y),'g',label = 'ang_vel_pitch')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('amplitude')
    plt.grid()
        
    plt.subplot(3, 2, 5)
    plt.plot(FT_freq,abs(new_ang_vel_z),'b',label = 'ang_vel_yaw')
    plt.legend(loc=(0.67, 0.8))
    plt.xlabel('Frequency[Hz]')
    plt.ylabel('amplitude')
    plt.grid()

    plt.subplot(3, 2, 2) 
    plt.plot(FT_freq,abs(new_acc_x),'r',label = 'lin_acc_x')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('amplitude')
    plt.title('self.imu_lin_acc[Freq-Domain]')        
    plt.grid()
       
    plt.subplot(3, 2, 4)
    plt.plot(FT_freq,abs(new_acc_y),'g',label = 'lin_acc_y')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('amplitude')
    plt.grid()

    plt.subplot(3, 2, 6)
    plt.plot(FT_freq,abs(new_acc_z),'b',label = 'lin_acc_z')
    plt.legend(loc=(0.67, 0.8))
    plt.ylabel('amplitude')
    plt.xlabel('Frequency[Hz]')
    plt.grid()    

    plt.show()

#____________________________________________________________________________

ra_files = sorted(get_filelist(data_path+'/', 'ra'))   # 파일 불러오기
for ra_filename in ra_files:                # save 6 Robot Motion
    ra_np = get_ra(ra_filename)


    imu_ang_vel_raw_x = ra_np[:,19]
    imu_ang_vel_raw_y = ra_np[:,20]
    imu_ang_vel_raw_z = ra_np[:,21]


    imu_lin_acc_x = ra_np[:,28]
    imu_lin_acc_y = ra_np[:,29]
    imu_lin_acc_z = ra_np[:,30]

time = [i/100 for i in range(len(imu_ang_vel_raw_x))]        # Our 3237 data sequence is 0.01sec

n = imu_lin_acc_x.size
t_gap = 0.01
FT_freq = np.fft.fftfreq(n, d=t_gap)                        # make Time to Frequence



#____________________________________________________________________________
# 실행하고싶은 함수 실행 과제1,2,3,4 = 각 Problem숫자()

Problem1()
Problem2()
Problem3_0()
Problem3()
























