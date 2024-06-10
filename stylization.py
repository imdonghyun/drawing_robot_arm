import os
import argparse

from PIL import Image
import numpy as np

import torch
from torchvision.transforms.functional import to_tensor, to_pil_image

from style_model import Generator


torch.backends.cudnn.enabled = False
torch.backends.cudnn.benchmark = False
torch.backends.cudnn.deterministic = True

checkpoint = './weights/celeba_distill.pt'
device = 'cpu'
input_dir = './cheese'
output_dir = './result'
upsample_align = False
x32 = False


def style_change(image_path, checkpoint, idx):
    device = 'cpu'

    net = Generator()
    net.load_state_dict(torch.load(checkpoint, map_location="cpu"))
    net.to(device).eval()
    print(f"model loaded: {checkpoint}")

    img = Image.open(image_path).convert("RGB")


    with torch.no_grad():
        image = to_tensor(img).unsqueeze(0) * 2 - 1
        out = net(image.to(device), upsample_align).cpu()
        out = out.squeeze(0).clip(-1, 1) * 0.5 + 0.5
        out = to_pil_image(out)

    out.save('captured_image_stylize_' + str(idx) + '_.jpg')


