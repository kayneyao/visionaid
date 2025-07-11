#!/usr/bin/env python3

import sys
import os

# Add the package to Python path
package_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'yolov8_bdd100k_detection')
sys.path.insert(0, package_path)

def main():
    from yolov8_camera_node import main as node_main
    node_main()

if __name__ == '__main__':
    main()
