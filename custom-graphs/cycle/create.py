import math
import os

# this is an example file to create cycle graphs.
# just run the file in this directory 
# and then choose the name "cycle" for the map of shape "From File".
# disclaimer: this code was created from pure vibes 
# (but stripped down and tested by me - a human).

def create_cycle_file(n):
    lines = []
    radius = 800

    for i in range(n):
        angle = 2 * math.pi * i / n
        x = int(round(radius * math.cos(angle)))
        y = int(round(radius * math.sin(angle)))
        lines.append(f"V{i}({x},{y})")

    for i in range(n):
        next_i = (i + 1) % n
        lines.append(f"E{i},{next_i}")

    filename = f"{n}.txt"
    with open(filename, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

if __name__ == "__main__":
    # creates cycles from size 3 to 100.
    for num_vertices in range(3, 100 + 1):
        create_cycle_file(num_vertices)
