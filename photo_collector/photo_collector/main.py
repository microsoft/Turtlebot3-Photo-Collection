#!/usr/bin/env python3

import rclpy
import sys
from photo_collector.photo_collector import PhotoCollector

def main(args=None):
    try:
        # Setup
        rclpy.init(args=args)
        photo_collector = PhotoCollector()
        photo_collector.state["sample_range"]=360
        photo_collector.run()
        
    except KeyboardInterrupt:
        print("Closing policy connection due to keyboard interrupt")
    finally:
        # Cleanup
        photo_collector.destroy_node()
        rclpy.shutdown()