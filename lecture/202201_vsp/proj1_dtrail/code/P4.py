import numpy as np
from numpy.linalg import inv
import glob
import csv  
from matplotlib import pyplot as plt
from math import cos, sin, asin, atan2, pi, tan
from scipy import io                         


data_path = '/workspace/202201_vsp/proj1_dtrail/db_rosbag'

def get_filelist(path, ext):
    filelist = [f for f in glob.glob(path + "*." + ext)]
    return filelist


def get_ra(ra_file):
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



def Do_FFT(signal_list):
    FFT_OUT_PUT = np.fft.fft(signal_list)
    amp = abs(FFT_OUT_PUT) 
    
    return FFT_OUT_PUT,amp

def Do_IFFT(signal_list):
    IFFT_OUT_PUT = np.fft.ifft(signal_list)

    return IFFT_OUT_PUT  


def freq_idx_line(any_dataset,FT_freq):
    L = len(any_dataset)
    m = L//2
    slope = FT_freq[m]/m
    return slope


def Change_Target_Hz2idx(freq_slope,hz):
    idx = hz/freq_slope
    return idx


def Do_LOW_PASS_Filter(F_domain_signal,Threshold): # Kill Threshold_Hz < freq  data
    for i in range(len(F_domain_signal)):
        if i < Threshold or i > len(F_domain_signal) - Threshold:
            F_domain_signal[i] = F_domain_signal[i]
        else:
            F_domain_signal[i] = 0
    return F_domain_signal


def Do_High_PASS_Filter(F_domain_signal,Threshold): # Kill Threshold_Hz < freq  data
    for i in range(len(F_domain_signal)):
        if i < Threshold or i > len(F_domain_signal) - Threshold:
            F_domain_signal[i] = 0
        else:
            F_domain_signal[i] = F_domain_signal[i]
    return F_domain_signal

def Do_BEND_PASS_Filter(F_domain_signal,Threshold1,Threshold2):   #Threshold1 > Threshold2
    a = F_domain_signal[0]
    b = F_domain_signal[-1]
    for i in range(len(F_domain_signal)):
        
        if i > Threshold1 and i < len(F_domain_signal) - Threshold1:
            F_domain_signal[i] = 0
        elif i < Threshold2 or i > len(F_domain_signal) - Threshold2:
            F_domain_signal[i] = 0
        else:
            F_domain_signal[i] = F_domain_signal[i]
    F_domain_signal[0] = a
    F_domain_signal[-1] = b
    return F_domain_signal


ra_files = sorted(get_filelist(data_path+'/', 'ra'))
for ra_filename in ra_files:
    ra_np     = get_ra(ra_filename)


    imu_ang_vel_raw_x = ra_np[:,19]
    imu_ang_vel_raw_y = ra_np[:,20]
    imu_ang_vel_raw_z = ra_np[:,21]


    imu_lin_acc_x = ra_np[:,28]
    imu_lin_acc_y = ra_np[:,29]
    imu_lin_acc_z = ra_np[:,30]

time = [i/100 for i in range(len(imu_ang_vel_raw_x))]

n = imu_lin_acc_x.size
t_gap = 0.01
FT_freq = np.fft.fftfreq(n, d=t_gap)



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

new1_ang_vel_x = Do_IFFT(new_ang_vel_x)
new1_ang_vel_y = Do_IFFT(new_ang_vel_y)
new1_ang_vel_z = Do_IFFT(new_ang_vel_z)

new1_acc_x = Do_IFFT(new_acc_x)
new1_acc_y = Do_IFFT(new_acc_y)
new1_acc_z = Do_IFFT(new_acc_z)



H, Q, R = None, None, None
x, P = None, None
firstRun = True

H2, Q2, R2 = None, None, None
x2, P2 = None, None
Secondrun = True

H3, Q3, R3 = None, None, None
x3, P3 = None, None
Thirdrun = True

def GetGyro(i):
    p = new1_ang_vel_x[i]  
    q = new1_ang_vel_y[i] 
    r = new1_ang_vel_z[i]
    return p, q, r

def GetAccel(i):
    ax = new1_acc_x[i]  
    ay = new1_acc_y[i]  
    az = new1_acc_z[i]  
    return ax, ay, az

def GetGyro2(i):                      # raw_data_function
    p = imu_ang_vel_raw_x[i]  
    q = imu_ang_vel_raw_y[i] 
    r = imu_ang_vel_raw_z[i] 
    return p, q, r

def GetAccel2(i):                     # raw_data_function
    ax = imu_lin_acc_x[i]  
    ay = imu_lin_acc_y[i]  
    az = imu_lin_acc_z[i]  
    return ax, ay, az

def EulerAccel(ax, ay, az):
    g = 9.8
    theta = asin(ax / g)
    phi = asin(-ay / (g * cos(theta)))
    return phi, theta

def sec(theta):
    return 1/cos(theta)

def Ajacob(xhat, rates, dt):

    A = np.zeros([3,3])
    phi = xhat[0]
    theta = xhat[1]

    p,q,r = rates[0], rates[1], rates[2]

    A[0][0] = q * cos(phi)*tan(theta) - r*sin(phi)*tan(theta)
    A[0][1] = q * sin(phi)*(sec(theta)**2) + r*cos(phi)*(sec(theta)**2)
    A[0][2] = 0

    A[1][0] = -q * sin(phi) - r * cos(phi)
    A[1][1] = 0
    A[1][2] = 0

    A[2][0] = q * cos(phi) * sec(theta) - r * sin(phi) * sec(theta)
    A[2][1] = q * sin(phi) * sec(theta)*tan(theta) + r*cos(phi)*sec(theta)*tan(theta)
    A[2][2] = 0

    A = np.eye(3) + A*dt

    return A

def fx(xhat, rates, dt):
    phi = xhat[0]
    theta = xhat[1]

    p,q,r = rates[0], rates[1], rates[2]

    xdot = np.zeros([3,1])
    xdot[0] = p + q * sin(phi) * tan(theta) + r * cos(phi)*tan(theta)
    xdot[1] = q * cos(phi) - r * sin(phi)
    xdot[2] = q * sin(phi)*sec(theta) + r * cos(phi) * sec(theta)

    xp = xhat.reshape(-1,1) + xdot*dt # xhat : (3,) --> (3,1)
    return xp



def EulerToQuaternion(phi, theta, psi):
    sinPhi = sin(phi/2)
    cosPhi = cos(phi/2)
    sinTheta = sin(theta/2)
    cosTheta = cos(theta/2)
    sinPsi = sin(psi/2)
    cosPsi = cos(psi/2)
    z = np.array([cosPhi*cosTheta*cosPsi + sinPhi*sinTheta*sinPsi,
                  sinPhi*cosTheta*cosPsi - cosPhi*sinTheta*sinPsi,
                  cosPhi*sinTheta*cosPsi + sinPhi*cosTheta*sinPsi,
                  cosPhi*cosTheta*sinPsi - sinPhi*sinTheta*cosPsi])
    return z

def EulerKalman(A2, z2):
    global Secondrun
    global Q2, H2, R2
    global x2, P2
    if Secondrun:
        H2 = np.eye(4)
        Q2 = 0.0001 * np.eye(4)
        R2 = 10 * np.eye(4)
        x2 = np.array([1, 0, 0, 0]).transpose()
        P2 = np.eye(4)
        Secondrun = False
    else:
        Xp2 = A2 @ x2 # Xp : State Variable Prediction
        Pp2 = A2 @ P2 @ A2.T + Q2 # Error Covariance Prediction

        K2 = (Pp2 @ H2.T) @ inv(H2@Pp2@H2.T + R2) # K : Kalman Gain

        x2 = Xp2 + K2@(z2 - H2@Xp2) # Update State Variable Estimation
        P2 = Pp2 - K2@H2@Pp2 # Update Error Covariance Estimation


    phi   = atan2(2 * (x2[2] * x2[3] + x2[0] * x2[1]), 1 - 2*(x2[1]**2 + x2[2]**2))
    theta = -asin(2 *  (x2[1] * x2[3] - x2[0] * x2[2]))
    psi   = atan2(2 *  (x2[1] * x2[2] + x2[0] * x2[3]), 1-2*(x2[2]**2 + x2[3]**2))


    return phi, theta, psi

def EulerKalman2(A, z):
    global Thirdrun
    global Q3, H3, R3
    global x3, P3
    if Thirdrun:
        H3 = np.eye(4)
        Q3 = 0.0001 * np.eye(4)
        R3 = 10 * np.eye(4)
        x3 = np.array([1, 0, 0, 0]).transpose()
        P3 = np.eye(4)
        Thirdrun = False
    else:
        Xp3 = A @ x3 # Xp : State Variable Prediction
        Pp3 = A @ P3 @ A.T + Q3 # Error Covariance Prediction

        K3 = (Pp3 @ H3.T) @ inv(H3@Pp3@H3.T + R3) # K : Kalman Gain

        x3 = Xp3 + K3@(z - H3@Xp3) # Update State Variable Estimation
        P3 = Pp3 - K3@H3@Pp3 # Update Error Covariance Estimation


    phi   = atan2(2 * (x3[2] * x3[3] + x3[0] * x3[1]), 1 - 2*(x3[1]**2 + x3[2]**2))
    theta = -asin(2 *  (x3[1] * x3[3] - x3[0] * x3[2]))
    psi   = atan2(2 *  (x3[1] * x3[2] + x3[0] * x3[3]), 1-2*(x3[2]**2 + x3[3]**2))


    return phi, theta, psi


def EulerEKF(z, rates, dt):
    global firstRun
    global Q, H, R
    global x, P
    if firstRun:
        H = np.array([[1,0,0],[0,1,0]])
        Q = np.array([[0.0001,0,0],[0,0.0001,0],[0,0,0.1]])
        R = 10 * np.eye(2)
        x = np.array([0, 0, 0]).transpose()
        P = 10 * np.eye(3)
        firstRun = False
    else:
        A = Ajacob(x, rates, dt)
        Xp = fx(x, rates, dt)                 # Xp : State Variable Prediction
        Pp = A @ P @ A.T + Q                  # Error Covariance Prediction

        K = (Pp @ H.T) @ inv(H@Pp@H.T + R)    # K : Kalman Gain

        x = Xp + K@(z.reshape(-1,1) - H@Xp)   # Update State Variable Estimation
        P = Pp - K@H@Pp                       # Update Error Covariance Estimation

    phi   = x[0]
    theta = x[1]
    psi   = x[2]
    return phi, theta, psi


Nsamples = len(imu_lin_acc_x)
dt = 0.01
EulerSaved = np.zeros([Nsamples,3])
EulerSaved2 = np.zeros([Nsamples,3])
EulerSaved3 = np.zeros([Nsamples,3])
t = np.arange(0, Nsamples * dt ,dt)



for k in range(Nsamples):
    p2, q2, r2 = GetGyro2(k)
    A2 = np.eye(4) + dt * (1/2) * np.array([[0,-p2,-q2,-r2],[p2,0,r2,-q2],[q2,-r2,0,p2],[r2,q2,-p2,0]])
    ax2, ay2, az2 = GetAccel2(k)
    phi2, theta2 = EulerAccel(ax2, ay2, az2)
    z2 = EulerToQuaternion(phi2, theta2, 0) 

    phi2, theta2, psi2 = EulerKalman(A2, z2)
    EulerSaved2[k] = [phi2, theta2, psi2]

    p, q, r = GetGyro(k)
    ax, ay, az = GetAccel(k)
    phi_a, theta_a = EulerAccel(ax, ay, az)
    phi, theta, psi = EulerEKF(np.array([phi_a, theta_a]).T, [p,q,r], dt)

    EulerSaved[k] = [phi, theta, psi]

    p3, q3, r3 = GetGyro(k)
    A3 = np.eye(4) + dt * (1/2) * np.array([[0,-p3,-q3,-r3],[p3,0,r3,-q3],[q3,-r3,0,p3],[r3,q3,-p3,0]])
    ax3, ay3, az3 = GetAccel(k)
    phi3, theta3 = EulerAccel(ax3, ay3, az3)
    z3 = EulerToQuaternion(phi3, theta3, 0) 

    phi3, theta3, psi3 = EulerKalman2(A3, z3)
    EulerSaved3[k] = [phi3, theta3, psi3]


PhiSaved = EulerSaved[:,0] * 180/pi
ThetaSaved = EulerSaved[:,1] * 180/pi
PsiSaved = EulerSaved[:,2] * 180/pi

PhiSaved2 = EulerSaved2[:,0] * 180/pi
ThetaSaved2 = EulerSaved2[:,1] * 180/pi
PsiSaved2 = EulerSaved2[:,2] * 180/pi

PhiSaved3 = EulerSaved3[:,0] * 180/pi
ThetaSaved3 = EulerSaved3[:,1] * 180/pi
PsiSaved3 = EulerSaved3[:,2] * 180/pi



# Plot Code

plt.figure(figsize=(16,8))                          # Compare 3 kinds of 'roll angle'
plt.subplot(1, 3, 1) 
plt.plot(t, PhiSaved2,'r',label = 'raw + KF')
plt.legend(loc=(0.67, 0.8))
plt.ylabel('Roll angel[deg]')
plt.xlabel('Time[Sec]')
plt.title('Reduce Noise (Roll)')
plt.grid()
plt.subplot(1, 3, 2) 
plt.plot(t, PhiSaved3,'g',label = 'LPF/BPF + KF')
plt.legend(loc=(0.67, 0.8))
plt.ylabel('Roll angel[deg]')
plt.title('Reduce Noise (Roll)')
plt.xlabel('Time[Sec]')
plt.grid()
plt.subplot(1, 3, 3) 
plt.plot(t, PhiSaved,'b',label = 'LPF/BPF + EKF')
plt.legend(loc=(0.67, 0.8))
plt.ylabel('Roll angel[deg]')
plt.xlabel('Time[Sec]')
plt.title('Reduce Noise (Roll)')
plt.grid()


plt.figure(figsize=(16,8))                         # Compare 3 kinds of 'Pitch angle'
plt.subplot(1, 3, 1) 
plt.plot(t, ThetaSaved2,'r',label = 'raw + KF')
plt.legend(loc=(0.67, 0.8))
plt.ylabel('Pitch angel[deg]')
plt.xlabel('Time[Sec]')
plt.title('Reduce Noise (Pitch)')
plt.grid()
plt.subplot(1, 3, 2) 
plt.plot(t, ThetaSaved3,'g',label = 'LPF/BPF + KF')
plt.legend(loc=(0.67, 0.8))
plt.ylabel('Pitch angel[deg]')
plt.xlabel('Time[Sec]')
plt.title('Reduce Noise (Pitch)')
plt.grid()
plt.subplot(1, 3, 3) 
plt.plot(t, ThetaSaved,'b',label = 'LPF/BPF + EKF')
plt.legend(loc=(0.67, 0.8))
plt.ylabel('Pitch angel[deg]')
plt.xlabel('Time[Sec]')
plt.title('Reduce Noise (Pitch)')
plt.grid()



plt.figure(figsize=(16,8))              # Compare 3 kinds of 'Yaw angle'
plt.subplot(1, 3, 1) 
plt.plot(t, PsiSaved2,'r',label = 'raw + KF')
plt.legend(loc=(0.67, 0.8))
plt.ylabel('Yaw angel[deg]')
plt.xlabel('Time[Sec]')
plt.title('Reduce Noise (Yaw)')
plt.grid()
plt.subplot(1, 3, 2)
plt.plot(t, PsiSaved3,'g',label = 'LPF/BPF + KF')
plt.legend(loc=(0.67, 0.8))
plt.ylabel('Yaw angel[deg]')
plt.xlabel('Time[Sec]')
plt.title('Reduce Noise (Yaw)')
plt.grid()
plt.subplot(1, 3, 3)
plt.plot(t, PsiSaved,'b',label = 'LPF/BPF + EKF')
plt.legend(loc=(0.67, 0.8))
plt.ylabel('Yaw angel[deg]')
plt.xlabel('Time[Sec]')
plt.title('Reduce Noise (Yaw)')
plt.grid()

plt.show()

