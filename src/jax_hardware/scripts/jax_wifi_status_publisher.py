#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import re
import time

class WifiStatusPublisher(Node):
    def __init__(self):
        super().__init__('wifi_status_publisher')
        self.publisher_ = self.create_publisher(String, 'jax/wifi_status', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ssid, bars = self.get_wifi_status()
        msg = String()
        msg.data = f"{ssid}|{bars}"
        self.publisher_.publish(msg)

    def get_wifi_status(self):
        try:
            out = subprocess.check_output(
                ["bash", "-lc", "iwconfig 2>/dev/null | grep -i --color=never 'Link Quality\\|ESSID' -m 1 || true"],
                universal_newlines=True,
                timeout=1.0
            )
        except Exception:
            return ("NO WIFI", 0)
        if "Link Quality=" in out:
            try:
                q = out.split("Link Quality=")[1].split()[0]
                num, denom = map(int, q.split("/"))
                quality = num / denom
                if quality > 0.8:
                    bars = 4
                elif quality > 0.6:
                    bars = 3
                elif quality > 0.4:
                    bars = 2
                elif quality > 0.2:
                    bars = 1
                else:
                    bars = 0
            except Exception:
                bars = 1
            ssid_match = re.search(r'ESSID:"([^"]+)"', out)
            ssid = ssid_match.group(1) if ssid_match else "WIFI"
            return (ssid, bars)
        return ("NO WIFI", 0)

def main(args=None):
    rclpy.init(args=args)
    node = WifiStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
