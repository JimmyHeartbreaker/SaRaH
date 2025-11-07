import pygame
import numpy as np
import math

SCREEN_SIZE = 1000
SCALE = 0.2
CENTER = (SCREEN_SIZE // 2, SCREEN_SIZE // 2)
FPS = 30

pygame.init()
screen = pygame.display.set_mode((SCREEN_SIZE, SCREEN_SIZE))
clock = pygame.time.Clock()


# --- Draw function ---
def draw_lidar_pixels(points,screen_array,color, robot_pos=(0,0)):
    # Create a fresh array each frame (safe)

    # Extract x and y
    x_coords = np.array([p.x for p in points], dtype=np.float32)
    y_coords = np.array([p.y for p in points], dtype=np.float32)

    # Filter NaNs
    mask = ~np.isnan(x_coords) & ~np.isnan(y_coords)
    x_coords = x_coords[mask]
    y_coords = y_coords[mask]

    # Convert to pixel coordinates
    px = np.round(CENTER[0] + (x_coords - robot_pos[0]) * SCALE).astype(np.int32)
    py = np.round(CENTER[1] - (y_coords - robot_pos[1]) * SCALE).astype(np.int32)

    # Clip to screen boundaries
    px = np.clip(px, 0, SCREEN_SIZE-1)
    py = np.clip(py, 0, SCREEN_SIZE-1)

    # Set pixels
    screen_array[px, py] = color # note axes order (row, col)

    # Blit once
    pygame.surfarray.blit_array(screen, screen_array)

# --- Main loop ---
def update():
    pygame.display.flip()
    clock.tick(FPS)
