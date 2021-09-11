import numpy as np
import torch

#load weights
W = np.load('weight.npy')
 
# B = np.load('bias.npy')
B = np.asarray([-1.1, -14.38])

f = lambda x, y: np.matmul(np.asarray([x, y])-B, W)
g = lambda x, y: np.matmul(np.asarray([x, y]), np.linalg.inv(W)) + B

## testing

Conversion = [f(-2.60, -5.17),
 f(-6.05, -5.75), 
 f(-7.93, 18.45), 
 f(-.23, 9.19),   
 f(-12.30, -6.28)]

Real = [[9.8, 0.0],
        [9.8, 3.5],
        [34., 1.5],
        [23.63, -4.62],
        [10.27, 9.76]]

diff = torch.nn.MSELoss()
Conversion = torch.as_tensor(Conversion)
Real = torch.as_tensor(Real)
res = diff(Conversion, Real)


