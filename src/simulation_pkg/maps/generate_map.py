#!/usr/bin/env python3
import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation
import os

RES = 0.05
W_M, H_M = 6.0, 6.0
W_PX = int(W_M / RES)
H_PX = int(H_M / RES)
MARGIN = 4

def m2px_col(x): return int(x / RES)
def m2px_row(y): return H_PX - int(y / RES) - 1

grid = np.full((H_PX, W_PX), 254, dtype=np.uint8)
grid[:MARGIN, :] = 0; grid[-MARGIN:, :] = 0
grid[:, :MARGIN] = 0; grid[:, -MARGIN:] = 0

# Obstacle 1: 2.5x0.9m at top-left
r1a, r1b = m2px_row(5.00), m2px_row(4.10)
c1a, c1b = m2px_col(0.30), m2px_col(2.80)
grid[min(r1a,r1b):max(r1a,r1b)+1, min(c1a,c1b):max(c1a,c1b)+1] = 0

# Obstacle 2: 1.4x2.0m at top-right
r2a, r2b = m2px_row(5.20), m2px_row(3.20)
c2a, c2b = m2px_col(4.20), m2px_col(5.60)
grid[min(r2a,r2b):max(r2a,r2b)+1, min(c2a,c2b):max(c2a,c2b)+1] = 0

inflated = binary_dilation(grid==0, iterations=4)
grid[inflated] = 0

out = os.path.join(os.path.dirname(__file__), 'shopfloor.pgm')
Image.fromarray(grid, mode='L').save(out)
print(f"Saved {out}  ({W_PX}x{H_PX} px)")
