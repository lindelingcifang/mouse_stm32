import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('D:\Desktop\ZJUNlict\mouse_stm32\Ozone_DataSampling_250706_2.csv')
vision = pd.read_csv('D:\Desktop\ZJUNlict\mouse_stm32\ddd4.csv')

x_vision = vision['x']
y_vision = vision['y']
yaw_vision = vision['yaw']

imu = df['((robot).imu)->data[10]']/180.0*np.pi

basis_imu = imu[0] - yaw_vision[0]

for i in range(len(imu)-1):
    imu[i] = imu[i] - basis_imu
    imu[i] = -imu[i]

x = df['debug_X']
y = df['debug_Y']
dx = x.copy()
dy = y.copy()

x = -x

x_new = [0]*len(dx)
y_new = [0]*len(dx)

for i in range(len(imu)-1):
    
    deltaImu = imu[i+1]-imu[i]
    
    if deltaImu > np.pi:
        deltaImu = deltaImu - 2 * np.pi
        
    if deltaImu < -np.pi:
        deltaImu = deltaImu + 2 * np.pi
    
    dx[i] = x[i+1] - x[i] - np.sqrt(118*118+13*13) * deltaImu
    dy[i] = y[i+1] - y[i] 
    
    dx_t = (dx[i]*np.cos(deltaImu-np.arctan2(13.0,113.0)) - dy[i]*np.sin(deltaImu-np.arctan2(13.0,113.0)))
    dy_t = (dx[i]*np.sin(deltaImu-np.arctan2(13.0,113.0)) + dy[i]*np.cos(deltaImu-np.arctan2(13.0,113.0)))

    dx_tt = dy_t*np.cos(imu[i]) + dx_t*np.sin(imu[i])
    dy_tt = -dy_t*np.sin(imu[i]) + dx_t*np.cos(imu[i])

    x_new[i+1] = x_new[i] + dx_tt
    y_new[i+1] = y_new[i] + dy_tt 

#     print(x_new[i+1] - x_new[i])
    
dx[len(dx)-1] = dx[len(dx)-1]


# for i in range(len(imu)):
#     len_ = np.sqrt(x_new[i]**2 + y_new[i]**2)
#     t = np.arctan2(y_new[i], x_new[i])
#     t = t + imu[0]
#     x_new[i] = len_*np.cos(t)
#     y_new[i] = len_*np.sin(t)


compose_x = x_vision[0] - x_new[0]
compose_y = y_vision[0] - y_new[0]

for i in range(len(imu)):
    x_new[i] = x_new[i] + compose_x
    y_new[i] = y_new[i] + compose_y

plt.figure(figsize=(10, 8))


plt.plot(x_new, y_new, 'b--', label='new')
plt.plot(x, y, 'g-', label='raw')
plt.plot(x_vision, y_vision, 'r-', label='Vision')

# plt.plot(imu)
# plt.plot(yaw_vision)


plt.title('Trajectory Comparison: IMU-Corrected vs Vision vs Original')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
# plt.axis('equal') 
plt.show()

# plt.figure(figsize=(10, 8))
# plt.plot(yaw_vision)
# plt.title('Trajectory Comparison: IMU-Corrected vs Vision vs Original')
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.legend()
# plt.grid(True)
# # plt.axis('equal') 
# plt.show()