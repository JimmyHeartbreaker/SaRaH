import sys, time, math, signal, threading
import matplotlib.pyplot as plt

from lidar_worker import start_lidar_proc, lidar_worker, shared, state_lock
from path_planner import AStarPlanner
from config import *

def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    proc = start_lidar_proc()
    worker = threading.Thread(target=lidar_worker, args=(proc,), daemon=True)
    worker.start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(7,7))
    plt.show(block=False)
    user_next = threading.Event()

    def on_key(event):
        if event.key == 'q':
            print("Quit requested.")
            if proc: proc.terminate()
            plt.close(); sys.exit(0)
        elif event.key == 'n':
            print("Next scan triggered.")
            user_next.set()

    fig.canvas.mpl_connect('key_press_event', on_key)

    visited_goals = set()

    print("Starting doorway exploration.")
    while True:
        print("Waiting for map...")
        while True:
            with state_lock:
                occ_map = shared["occupancy_map"]
                cx = shared["center_x"]
                cy = shared["center_y"]
                clustered = list(shared["clustered_doors"])
            if occ_map is not None:
                break
            time.sleep(0.2)

        ax.clear()
        ax.imshow(occ_map.T, cmap="gray_r", origin="lower")
        ax.plot(cx, cy, "og")
        for (gx, gy, mx, my, gap) in clustered:
            ax.plot(gx, gy, "yo")
            ax.text(gx + 1, gy + 1, f"{gap:.2f}m", fontsize=6, color='y')
        plt.pause(0.1)

        if not clustered:
            print("No doorways found.")
            user_next.clear()
            while not user_next.is_set(): plt.pause(0.1)
            with state_lock: shared["occupancy_map"] = None
            continue

        doors = sorted([(math.hypot(mx,my), gx, gy, mx, my, gap) for gx, gy, mx, my, gap in clustered])
        next_goal = None
        for dist, gx, gy, mx, my, gap in doors:
            key = round(mx,2), round
if __name__ == "__main__":
    main()
