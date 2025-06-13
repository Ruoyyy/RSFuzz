import scipy.io as sio
mat_data = sio.loadmat('hard_obstacles.mat')
print(mat_data['obstacles'])