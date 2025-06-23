#!/usr/bin/env python3

import sys
import os
import rospy
import argparse

# Allow Python to find the package-level modules
current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from core.app_runner import AppRunner
from core.logger import init_logger

def main():
    # CLI args
    parser = argparse.ArgumentParser(description="ROS 1 MiddleMan single-drone server")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    args, _ = parser.parse_known_args()

    # ROS node init
    rospy.init_node("middle_man_node", anonymous=False)
    print(">>> ROS node initialized", flush=True)

    # Logger setup AFTER rospy.init_node()
    logger = init_logger(debug=args.debug)
    logger.info("[Main] Logger initialized")
    logger.info("[Main] Debug mode enabled" if args.debug else "[Main] Running in normal mode")

    # App startup
    app = AppRunner(logger)
    logger.info("[Main] AppRunner instance created")

    app = AppRunner(logger)
    app.start()

    rate = rospy.Rate(1.0)  # 1 Hz
    try:
        while not rospy.is_shutdown():
            app.tick()
            rate.sleep()
    except rospy.ROSInterruptException:
        logger.info("[Main] Caught shutdown signal.")

    app.shutdown()


if __name__ == "__main__":
    main()
