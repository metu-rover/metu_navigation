from detecto import core, utils, visualize
from detecto.visualize import detect_video

import cv2
import torch

model = core.Model.load('model_weights.pth', ['mavi_priz', 'main_switch', 'voltage_knob'])

visualize.detect_live(model)
