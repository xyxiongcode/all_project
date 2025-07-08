import torch
import numpy as np

def reconstruct_path(start_xy, directions, step=1):
    path = [start_xy]
    direction_vectors = [
        (-1, 0), (1, 0), (0, 1), (0, -1),
        (-1, 1), (-1, -1), (1, 1), (1, -1)
    ]
    for d in directions:
        dx, dy = direction_vectors[d]
        last = path[-1]
        path.append((last[0] + dx * step, last[1] + dy * step))
    return path
