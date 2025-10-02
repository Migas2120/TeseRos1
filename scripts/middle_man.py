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
    parser.add_argument("--debug", action="store_false", help="Enable debug logging")
    parser.add_argument("--ip",    required=True,        help="Unity host IP address")
    parser.add_argument("--unsafe", action="store_true",  help="Run in UNSAFE mode (bypass safety checks)")
    args, _ = parser.parse_known_args()

    # ROS node init
    rospy.init_node("middle_man_node", anonymous=False)
    print(">>> ROS node initialized", flush=True)

    # Logger setup AFTER rospy.init_node()
    logger = init_logger(debug=args.debug)
    logger.info("[Main] Logger initialized")
    logger.info("[Main] Debug mode enabled" if args.debug else "[Main] Running in normal mode")
    logger.warning("[Main] UNSAFE MODE ENABLED — safety checks will be bypassed!") if args.unsafe else logger.info("[Main] Safe mode active")

    # App startup
    app = AppRunner(logger, unsafe=args.unsafe)
    logger.info("[Main] AppRunner instance created")

    app.start()

    UNITY_PORT = 12345
    logger.info(f"[Main] Connecting out to Unity at {args.ip}:{UNITY_PORT}")
    app._init_client(host=args.ip, port=UNITY_PORT)

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
