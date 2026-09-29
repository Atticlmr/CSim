"""Check a viewer P6 screenshot without a display, Pillow, or exact pixel hashes."""
import sys
from pathlib import Path

with Path(sys.argv[1]).open("rb") as file:
    if file.readline() != b"P6\n":
        raise RuntimeError("Expected a P6 image")
    width, height = map(int, file.readline().split())
    if file.readline() != b"255\n":
        raise RuntimeError("Expected 8-bit channels")
    pixels = file.read()
if width < 640 or height < 360 or len(pixels) != width*height*3:
    raise RuntimeError("Invalid screenshot dimensions or storage")
# Sample the central scene region for drone and payload geometry.
cyan = gold = 0
for y in range(height//8, height*7//8):
    for x in range(width//3, width*9//10):
        r, g, b = pixels[(y*width+x)*3:(y*width+x)*3+3]
        cyan += g > 80 and b > 60 and g > 1.3*r
        gold += r > 80 and g > 40 and r > 1.1*g and g > 1.4*b
if cyan < 15 or gold < 15:
    raise RuntimeError(f"Missing scene geometry: cyan={cyan}, gold={gold}")
print(f"Rendered geometry verified: {width}x{height}, cyan={cyan}, gold={gold}")
