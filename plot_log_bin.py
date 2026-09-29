#!/usr/bin/env python3
# Usage: python plot_log_bin.py dctllog0.bin

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

log_size = 150

filename = sys.argv[1]
data = np.fromfile(filename, dtype=np.float32)
if len(data) % log_size != 0:
    raise ValueError(f"File size {len(data)} is not compatible")
records = data.reshape((-1, log_size))
N = records.shape[0]
print(f"Loaded: {filename}  |  {N} records")

# =============================================================================
fps_main    = np.clip(records[:, 0], 0, 120)
fps_th1     = np.clip(records[:, 1], 0, 120)
fps_th2     = np.clip(records[:, 2], 0, 20)
fps_imu     = np.clip(records[:, 3], 0, 120)
fps_of      = np.clip(records[:, 4], 0, 20)
fps_alt     = np.clip(records[:, 5], 0, 60)
fps_tof1    = np.clip(records[:, 6], 0, 20)
fps_adc     = np.clip(records[:, 7], 0, 120)
cpu         = np.clip(records[:, 8], 0, 120) # %
ram         = np.clip(records[:, 9], 0, 120) # %
temp        = np.clip(records[:, 10], 0, 120) # degC
roll        = records[:, 11] # deg
pitch       = records[:, 12] # deg
yaw         = records[:, 13] # deg
flow_x      = records[:, 14] # px
flow_y      = records[:, 15] # px
alt_of      = records[:, 16] # m
voltage     = records[:, 17] # V
tof1_avg    = records[:, 18] # m
t           = records[:, 19] # s
pos_ned_x   = records[:, 20] # m
pos_ned_y   = records[:, 21] # m
pos_ned_z   = records[:, 22] # m
pos_neu_x   = records[:, 23] # m
pos_neu_y   = records[:, 24] # m
pos_neu_z   = records[:, 25] # m
vel_ned_x   = records[:, 26] # m/s
vel_ned_y   = records[:, 27] # m/s
vel_ned_z   = records[:, 28] # m/s
vel_roll    = records[:, 29] # deg/s
vel_pitch   = records[:, 30] # deg/s
vel_yaw     = records[:, 31] # deg/s
dctl_stages = records[:, 32] # stages
flight_time = records[:, 33] # s
rc_cmd_roll  = records[:, 34] # roll
rc_cmd_pitch = records[:, 35] # pitch
rc_cmd_thr   = records[:, 36] # throttle
rc_cmd_yaw   = records[:, 37] # yaw
rc_cmd_arm   = records[:, 38] # arm
rc_cmd_angle = records[:, 39] # angle
rc_cmd_beep  = records[:, 40] # beep
# ToF8x8 down 41~104
tof8x8_down_m = records[:, 41:105] # print(tof8x8_down_m.shape)
# Controller settings
P_ned_vz      = records[:, 105]
I_ned_vz      = records[:, 106]
D_ned_vz      = records[:, 107]
PID_ned_vz    = records[:, 108]
tgt_ned_vz    = records[:, 109]
P_neu_z      = records[:, 110]
I_neu_z      = records[:, 111]
D_neu_z      = records[:, 112]
PID_neu_z    = records[:, 113]
tgt_neu_z    = records[:, 114]
P_ned_vx      = records[:, 115]
I_ned_vx      = records[:, 116]
D_ned_vx      = records[:, 117]
PID_ned_vx    = records[:, 118]
tgt_ned_vx    = records[:, 119]
P_ned_vy      = records[:, 120]
I_ned_vy      = records[:, 121]
D_ned_vy      = records[:, 122]
PID_ned_vy    = records[:, 123]
tgt_ned_vy    = records[:, 124]
P_neu_x      = records[:, 125]
I_neu_x      = records[:, 126]
D_neu_x      = records[:, 127]
PID_neu_x    = records[:, 128]
tgt_neu_x    = records[:, 129]
P_neu_y      = records[:, 130]
I_neu_y      = records[:, 131]
D_neu_y      = records[:, 132]
PID_neu_y    = records[:, 133]
tgt_neu_y    = records[:, 134]
P_yaw    = records[:, 135]
I_yaw    = records[:, 136]
D_yaw    = records[:, 137]
PID_yaw  = records[:, 138]
tgt_yaw  = records[:, 139]
# ground effect
ge_thrust    = records[:, 140]
ge_roll      = records[:, 141]
ge_pitch     = records[:, 142]
ge_yaw       = records[:, 143]
ge_alpha     = records[:, 144]
ge_h_eff     = records[:, 145]
ge_e_rms     = records[:, 146]
ge_dT_ff     = records[:, 147]

# =============================================================================

fig = plt.figure(figsize=(40, 24))
fig.suptitle(f"{filename}, Flighttime:{flight_time[-1]:.1f}s ({N} records)", fontsize=13, fontweight='bold')
gs = gridspec.GridSpec(3, 4, figure=fig, hspace=0.5, wspace=0.35) #rows,cols

def ax(row, col, colspan=1, rowspan=1):
    return fig.add_subplot(gs[row:row+rowspan, col:col+colspan])

# (0,0) FPS
a = ax(0,0)
a.plot(t, fps_main, label=f'Main: {fps_main.mean():.0f}', linewidth=1)
a.plot(t, fps_th1, label=f'Thread1: {fps_th1.mean():.0f}', linewidth=1)
a.plot(t, fps_th2, label=f'Thread2: {fps_th2.mean():.0f}', linewidth=1)
a.plot(t, fps_imu, label=f'IMU: {fps_imu.mean():.0f}', linewidth=1)
a.plot(t, fps_of, label=f'OF: {fps_of.mean():.0f}', linewidth=1)
a.plot(t, fps_alt, label=f'Alt: {fps_alt.mean():.0f}', linewidth=1)
a.plot(t, fps_tof1, label=f'ToF1: {fps_tof1.mean():.0f}', linewidth=1)
a.plot(t, fps_adc, label=f'ADC: {fps_adc.mean():.0f}', linewidth=1)
a.set_title('Threads')
a.set_ylabel('Hz')
a.set_xlabel('t [s]')
a.legend(fontsize=8)
a.grid(True, which='both', alpha=0.3)
a.minorticks_on()

# (0,1) CPU-RAM-Temp
a = ax(0,1)
a.plot(t, cpu, label=f'CPU: {cpu.max():.0f}%', linewidth=1, color='red')
a.plot(t, ram, label=f'RAM: {ram.max():.0f}%', linewidth=1, color='green')
a.set_ylabel('%', color='black')
a.set_title('CPU (Quadcore 512MB RAM)')
a.set_xlabel('t [s]')
a.grid(True, which='both', alpha=0.3)
a.minorticks_on()
a2 = a.twinx()
a2.plot(t, temp, label=f'Temp: {temp.max():.0f}°C', linewidth=1, color='blue')
a2.set_ylabel('°C', color='blue')
a2.tick_params(axis='y', labelcolor='blue')
lines1, labels1 = a.get_legend_handles_labels()
lines2, labels2 = a2.get_legend_handles_labels()
a.legend(lines1 + lines2, labels1 + labels2, fontsize=8)

# (0,2) Input CMD
a = ax(0,2)
a.plot(t, rc_cmd_roll, label='Roll', linewidth=1, color='red')
a.plot(t, rc_cmd_pitch, label='Pitch', linewidth=1, color='green')
a.plot(t, rc_cmd_yaw, label='Yaw', linewidth=1, color='blue')
a.plot(t, rc_cmd_thr, label='Throttle', linewidth=2, color='black')
a.set_ylabel('Value')
a.set_xlabel('t [s]')
a.set_title('Input CMD')
a.legend(fontsize=8)
a.grid(True, which='both', alpha=0.3)
a.minorticks_on()

# (0,3) Euler
a = ax(0,3)
a.plot(t, roll,  label='Roll',  linewidth=1, color='red')
a.plot(t, pitch, label='Pitch', linewidth=1, color='green')
a.plot(t, yaw,   label='Yaw',   linewidth=1, color='blue')
a.set_title('Euler (ENCcw)')
a.set_ylabel('Degree')
a.set_xlabel('t [s]')
a.legend(fontsize=8)
a.grid(True, which='both', alpha=0.3)
a.minorticks_on()

# (1,0) Optical flow raw
a = ax(1,0)
a.plot(t, flow_x, label='Flow X (lft)', linewidth=1, color='red')
a.plot(t, flow_y, label='Flow Y (fwd)', linewidth=1, color='green')
a.set_ylabel('px', color='black')
a.set_xlabel('t [s]')
a.set_title('Opticalflow (Raw)')
a.grid(True, which='both', alpha=0.3)
a.minorticks_on()
a2 = a.twinx()
#a2.plot(t, alt_of, label='Altitude', linewidth=1.2, color='blue')
a2.plot(t, tof1_avg, label='ToF1', linewidth=1, color='blue')
a2.set_ylabel('m', color='blue')
a2.tick_params(axis='y', labelcolor='blue')
lines1, labels1 = a.get_legend_handles_labels()
lines2, labels2 = a2.get_legend_handles_labels()
a.legend(lines1 + lines2, labels1 + labels2, fontsize=8)

# (1,1) Angular Velocity (NED)
a = ax(1,1)
a.plot(t, vel_roll, label='vroll', linewidth=1, color='red')
a.plot(t, vel_pitch, label='vpitch', linewidth=1, color='green')
a.plot(t, vel_yaw, label='vyaw', linewidth=1, color='blue')
a.set_title('Ang. Velocity (NED)')
a.set_ylabel('deg/s')
a.set_xlabel('t [s]')
a.legend(fontsize=8)
a.grid(True, which='both', alpha=0.3)
a.minorticks_on()

# (1,2) Linear Velocity (NED)
a = ax(1,2)
a.plot(t, vel_ned_x, label='vx', linewidth=1, color='red')
a.plot(t, vel_ned_y, label='vy', linewidth=1, color='green')
a.plot(t, vel_ned_z, label='vz', linewidth=1, color='blue')
a.set_title('Lin. Velocity (NED)')
a.set_ylabel('m/s')
a.set_xlabel('t [s]')
a.legend(fontsize=8)
a.grid(True, which='both', alpha=0.3)
a.minorticks_on()

# (1,3) Pos NED
a = ax(1,3)
a.plot(t, pos_ned_x, label='x', linewidth=1, color='red')
a.plot(t, pos_ned_y, label='y', linewidth=1, color='green')
a.plot(t, pos_ned_z, label='z', linewidth=1, color='blue')
a.set_title('Position (NED)')
a.set_ylabel('m')
a.set_xlabel('t [s]')
a.legend(fontsize=8)
a.grid(True, which='both', alpha=0.3)
a.minorticks_on()

# (2,0) Input CMD2 + stages
a = ax(2,0)
a.plot(t, dctl_stages, label='Stages', linewidth=1, color='red')
a.set_ylabel('Stages [0~4]', color='red')
a.tick_params(axis='y', labelcolor='red')
a.set_xlabel('t [s]')
a.set_title('Stages & Battery')
a.grid(True, which='both', alpha=0.3)
a.minorticks_on()
a2 = a.twinx()
a2.plot(t, voltage, label=f'Max: {voltage.max():.2f}\n Avg: {voltage.mean():.2f}\n Min: {voltage.min():.2f}', linewidth=1.5, color='blue')
a2.set_ylabel('Volt [V]', color='blue')
a2.tick_params(axis='y', labelcolor='blue')
lines1, labels1 = a.get_legend_handles_labels()
lines2, labels2 = a2.get_legend_handles_labels()
a.legend(lines1 + lines2, labels1 + labels2, fontsize=8)

# (2,1) 3D pose
a = fig.add_subplot(gs[2,1], projection='3d')
a.plot(pos_neu_y, pos_neu_x, pos_neu_z, linewidth=1, color='red')
a.scatter(pos_neu_y[0],  pos_neu_x[0],  pos_neu_z[0],  color='black', s=30, zorder=5, label='Start')
a.scatter(pos_neu_y[-1], pos_neu_x[-1], pos_neu_z[-1], color='green',  s=30, zorder=5, label='End')
a.set_xlabel('y [m]')
a.set_ylabel('x [m]')
a.set_zlabel('z [m]')
a.set_aspect('equal')
a.set_title('3D Pose (NEU)')
#a.legend(fontsize=8)
a.grid(True, which='both', alpha=0.3)
a.minorticks_on()

# (2,2) 2D Pose
a = ax(2,2)
a.plot(pos_neu_y, pos_neu_x, linewidth=1, color='cyan')  # y=horizontal, x=vertical
a.scatter(pos_neu_y[0],  pos_neu_x[0],  color='black', s=30, zorder=5, label='Start')
a.scatter(pos_neu_y[-1], pos_neu_x[-1], color='green', s=30, zorder=5, label='End')
a.set_xlabel('y [m]')
a.set_ylabel('x [m]')
a.set_aspect('equal')
a.set_title('2D Pose (NEU)')
#a.legend(fontsize=8)
a.grid(True, which='both', alpha=0.3)
a.minorticks_on()

# (2,3) Pos NEU
a = ax(2,3)
a.plot(t, pos_neu_x, label='x', linewidth=1, color='red')
a.plot(t, pos_neu_y, label='y', linewidth=1, color='green')
a.plot(t, pos_neu_z, label='z', linewidth=1, color='blue')
a.set_title('Position (NEU)')
a.set_ylabel('m')
a.set_xlabel('t [s]')
a.legend(fontsize=8)
a.grid(True, which='both', alpha=0.3)
a.minorticks_on()

#======================================================================================================================

k = 10  # frame to plot
tof = tof8x8_down_m[k].reshape(8, 8)
x, y = np.meshgrid(np.arange(8), np.arange(8))
x = x.flatten()
y = y.flatten()
z = tof.flatten()
fig0 = plt.figure(figsize=(10, 8))
ax0 = fig0.add_subplot(111, projection='3d')
ax0.scatter(
    x, y, z,
    c='red',
    s=50,
    marker='o'
)
ax0.set_box_aspect([1, 1, 1])
ax0.set_xlabel('x [px]')
ax0.set_ylabel('y [px]')
ax0.set_zlabel('Distance [m]')
ax0.set_title(f'ToF 8×8 Down (frame:{k})')

#======================================================================================================================

plt.rcParams['axes.grid'] = True
plt.rcParams['axes.grid.which'] = 'both'
plt.rcParams['grid.alpha'] = 0.3
plt.show()