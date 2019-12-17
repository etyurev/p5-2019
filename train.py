import numpy as np
import sys
sys.path.insert(0, 'include/')
import os
from tqdm import tqdm
import matplotlib.pyplot as plt
from net import Net

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

# Declare neural network'
if torch.cuda.is_available() == True:
    device = torch.device("cuda:0")  # you can continue going on here, like cuda:1 cuda:2....etc.
    print("Running on the GPU")
    net = Net(MAKE_TRAINING_DATA = False, VALIDATION_PROPORTION = 0.1, device=device).to(device)
    #net = Net()
    #net.to(device)
    #net.cuda()
else:
    device = torch.device("cpu")
    print("Running on the CPU")
    net = Net()

#net = Net(MAKE_TRAINING_DATA = False, VALIDATION_PROPORTION = 0.1)

net.train(patience = 6)
net.plot_training_results()
