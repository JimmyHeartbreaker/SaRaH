import matplotlib.pyplot as plt
import pandas as pd
import os
df = pd.read_csv(os.getcwd()+"\\tests\\new_lidar_points.csv")
plt.figure(figsize=(8,8))
plt.scatter(df['X'], df['Y'], s=5)
plt.gca().set_aspect('equal', adjustable='box')
plt.title("new LiDAR Points - N Shape")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.show()

df = pd.read_csv(os.getcwd()+"\\tests\\old_lidar_points.csv")
plt.figure(figsize=(8,8))
plt.scatter(df['X'], df['Y'], s=5)
plt.gca().set_aspect('equal', adjustable='box')
plt.title("old LiDAR Points - N Shape")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.show()