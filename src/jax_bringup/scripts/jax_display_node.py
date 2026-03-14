#!/usr/bin/env python3

import math
import subprocess
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import os
import platform
from PIL import Image
from luma.core.interface.serial import spi
from luma.lcd.device import st7789


@dataclass
class DisplayState:
    mode: str = "BOOT"
    battery_percent: int = 100
    battery_voltage: float = 16.8
    wifi_text: str = "N/A"
    wifi_bars: int = 0
    imu_ok: bool = False
    ros_ok: bool = True
    cpu_temp_c: float = 0.0
    sim: bool = True
    status_text: str = "STARTING"


class DesktopDisplayBackend:
    def __init__(self, window_name="Jax LCD Preview", scale=3):
        self.window_name = window_name
        self.scale = scale
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def show(self, img_bgr: np.ndarray):
        h, w = img_bgr.shape[:2]
        preview = cv2.resize(
            img_bgr,
            (w * self.scale, h * self.scale),
            interpolation=cv2.INTER_NEAREST
        )
        cv2.imshow(self.window_name, preview)
        cv2.waitKey(1)

    def close(self):
        cv2.destroyAllWindows()


class WaveshareDisplayBackend:
    def __init__(self, spi_port=0, spi_device=0, dc_pin=25, rst_pin=27):
        serial = spi(
            port=spi_port,
            device=spi_device,
            gpio_DC=dc_pin,
            gpio_RST=rst_pin
        )

        # The UI renderer already outputs 320x172 landscape.
        # Initialize the panel to match that orientation directly.
        self.device = st7789(
            serial,
            width=320,
            height=172,
            rotate=0
        )

    def show(self, img_bgr: np.ndarray):
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(img_rgb)
        self.device.display(pil_img)

    def close(self):
        pass


class JaxDisplayNode(Node):
    def __init__(self):
        super().__init__('jax_display_node')

        # ---------------- Parameters ----------------
        self.declare_parameter("mode_topic", "/jax_mode")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("sim", True)
        self.declare_parameter("demo_battery_drain", False)
        self.declare_parameter("battery_voltage_full", 16.8)
        self.declare_parameter("battery_voltage_empty", 13.2)
        self.declare_parameter("refresh_hz", 10.0)
        self.declare_parameter("robot_name", "JAX")
        self.declare_parameter("boot_duration", 2.5)
        self.declare_parameter("mode_flash_duration", 1.2)
        self.declare_parameter("use_lcd", False)
        self.declare_parameter("spi_port", 0)
        self.declare_parameter("spi_device", 0)
        self.declare_parameter("dc_pin", 25)
        self.declare_parameter("rst_pin", 27)

        mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        self.sim_mode = self.get_parameter("sim").get_parameter_value().bool_value
        self.demo_battery_drain = self.get_parameter("demo_battery_drain").get_parameter_value().bool_value
        self.v_full = self.get_parameter("battery_voltage_full").get_parameter_value().double_value
        self.v_empty = self.get_parameter("battery_voltage_empty").get_parameter_value().double_value
        refresh_hz = self.get_parameter("refresh_hz").get_parameter_value().double_value
        self.robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        self.boot_duration = self.get_parameter("boot_duration").get_parameter_value().double_value
        self.mode_flash_duration = self.get_parameter("mode_flash_duration").get_parameter_value().double_value
        use_lcd_param = self.get_parameter("use_lcd").get_parameter_value().bool_value
        spi_port = self.get_parameter("spi_port").get_parameter_value().integer_value
        spi_device = self.get_parameter("spi_device").get_parameter_value().integer_value
        dc_pin = self.get_parameter("dc_pin").get_parameter_value().integer_value
        rst_pin = self.get_parameter("rst_pin").get_parameter_value().integer_value

        # ---------------- State ----------------
        self.state = DisplayState()
        self.state.sim = self.sim_mode
        self.state.status_text = "STARTING"

        self.last_imu_time = 0.0
        self.start_time = time.time()
        self.demo_voltage = self.v_full

        # ---------------- Mode flash state ----------------
        self.current_mode = "BOOT"
        self.last_flash_mode = None
        self.flash_mode = None
        self.flash_until = 0.0

        # ---------------- Subscribers ----------------
        self.mode_sub = self.create_subscription(
            String,
            mode_topic,
            self.mode_cb,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_cb,
            10
        )

        # ---------------- Backend ----------------
        on_pi = (
            os.path.exists("/dev/spidev0.0") and
            platform.machine() in ["aarch64", "armv7l", "armv6l"]
        )

        try:
            if use_lcd_param or on_pi:
                self.get_logger().info("Using Waveshare LCD backend")
                self.backend = WaveshareDisplayBackend(
                    spi_port=spi_port,
                    spi_device=spi_device,
                    dc_pin=dc_pin,
                    rst_pin=rst_pin
                )
            else:
                self.get_logger().info("Using desktop preview backend")
                self.backend = DesktopDisplayBackend()
        except Exception as e:
            self.get_logger().warn(f"LCD init failed, falling back to desktop preview: {e}")
            self.backend = DesktopDisplayBackend()

        # ---------------- Timer ----------------
        period = 1.0 / max(refresh_hz, 1.0)
        self.timer = self.create_timer(period, self.update)

        self.get_logger().info("Jax display node started")
        self.get_logger().info(f"Mode topic: {mode_topic}")
        self.get_logger().info(f"IMU topic:  {imu_topic}")

    # ============================================================
    # Callbacks
    # ============================================================

    def mode_cb(self, msg: String):
        new_mode = msg.data.strip().upper()
        if not new_mode:
            new_mode = "UNKNOWN"

        old_mode = self.current_mode
        self.current_mode = new_mode
        self.state.mode = new_mode

        now = time.time()

        # Don't flash BOOT repeatedly, and don't retrigger if mode didn't change
        if new_mode != old_mode and new_mode != "BOOT":
            self.flash_mode = new_mode
            self.flash_until = now + self.mode_flash_duration
            self.last_flash_mode = new_mode

    def imu_cb(self, msg: Imu):
        _ = msg
        self.last_imu_time = time.time()
        self.state.imu_ok = True

    # ============================================================
    # Main update
    # ============================================================

    def update(self):
        now = time.time()

        # IMU freshness
        self.state.imu_ok = (now - self.last_imu_time) < 1.0

        # Battery
        self.update_battery(now)

        # Wi-Fi
        self.state.wifi_text, self.state.wifi_bars = self.get_wifi_status()

        # CPU temp
        self.state.cpu_temp_c = self.get_cpu_temp()

        # ROS health / footer
        self.state.ros_ok = rclpy.ok()
        self.state.status_text = self.build_status_text()

        # Render priority:
        # 1. boot screen
        # 2. mode flash
        # 3. normal dashboard
        if (now - self.start_time) < self.boot_duration:
            img = self.render_boot_screen(now - self.start_time)
        elif self.flash_mode is not None and now < self.flash_until:
            remaining = max(0.0, self.flash_until - now)
            img = self.render_mode_flash(self.flash_mode, remaining)
        else:
            img = self.render_dashboard(self.state)

        self.backend.show(img)

    # ============================================================
    # Helpers
    # ============================================================

    def update_battery(self, now: float):
        if self.demo_battery_drain:
            elapsed = now - self.start_time
            self.demo_voltage = max(self.v_empty, self.v_full - 0.0025 * elapsed)
        else:
            wiggle = 0.08 * math.sin(now * 1.2)
            self.demo_voltage = float(np.clip(15.8 + wiggle, self.v_empty, self.v_full))

        pct = self.voltage_to_percent(self.demo_voltage, self.v_empty, self.v_full)
        self.state.battery_voltage = float(self.demo_voltage)
        self.state.battery_percent = int(pct)

    @staticmethod
    def voltage_to_percent(voltage: float, v_empty: float, v_full: float) -> float:
        if v_full <= v_empty:
            return 0.0
        pct = 100.0 * (voltage - v_empty) / (v_full - v_empty)
        return max(0.0, min(100.0, pct))

    def get_wifi_status(self):
        try:
            out = subprocess.check_output(
                ["bash", "-lc", "iwconfig 2>/dev/null | grep -i --color=never 'Link Quality\\|ESSID' -m 1 || true"],
                text=True
            ).strip()

            if not out:
                return "NO WIFI", 0

            if "Link Quality=" in out:
                try:
                    q = out.split("Link Quality=")[1].split()[0]
                    a, b = q.split("/")
                    a = float(a)
                    b = float(b)
                    ratio = a / b if b > 0 else 0.0

                    if ratio > 0.80:
                        bars = 4
                    elif ratio > 0.60:
                        bars = 3
                    elif ratio > 0.35:
                        bars = 2
                    elif ratio > 0.10:
                        bars = 1
                    else:
                        bars = 0

                    return "WIFI", bars
                except Exception:
                    return "WIFI?", 1

            return "WIFI?", 1
        except Exception:
            return "NO WIFI", 0

    def get_cpu_temp(self):
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="utf-8") as f:
                raw = f.read().strip()
            return float(raw) / 1000.0
        except Exception:
            pass

        try:
            out = subprocess.check_output(
                ["bash", "-lc", "vcgencmd measure_temp 2>/dev/null || true"],
                text=True
            ).strip()
            if "temp=" in out:
                val = out.split("temp=")[1].split("'")[0]
                return float(val)
        except Exception:
            pass

        return 0.0

    def build_status_text(self):
        alerts = []

        if self.state.battery_percent <= 15:
            alerts.append("LOW BAT")

        if not self.state.imu_ok:
            alerts.append("IMU LOST")

        if self.state.wifi_bars == 0:
            alerts.append("WIFI LOST")

        if self.state.cpu_temp_c >= 75.0:
            alerts.append("HOT")

        if self.state.sim:
            alerts.append("SIM")

        if not alerts:
            return "ROS OK"

        return " | ".join(alerts)

    # ============================================================
    # Rendering
    # ============================================================

    def render_boot_screen(self, t: float) -> np.ndarray:
        width = 320
        height = 172
        img = np.zeros((height, width, 3), dtype=np.uint8)

        BG = (10, 10, 10)
        PANEL = (24, 24, 24)
        BORDER = (70, 70, 70)
        WHITE = (240, 240, 240)
        LIGHT = (170, 170, 170)
        CYAN = (220, 220, 60)
        GREEN = (60, 220, 90)

        img[:] = BG
        cv2.rectangle(img, (2, 2), (width - 3, height - 3), BORDER, 1)
        cv2.rectangle(img, (14, 18), (width - 15, height - 19), PANEL, -1)

        pulse = 0.5 + 0.5 * math.sin(2.5 * math.pi * t)
        line_w = int(220 * pulse)
        cv2.rectangle(img, (50, 34), (50 + line_w, 40), CYAN, -1)

        cv2.putText(
            img,
            self.robot_name[:10],
            (95, 86),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.65,
            WHITE,
            3,
            cv2.LINE_AA
        )

        subtitle = "INITIALIZING"
        if self.state.sim:
            subtitle = "INITIALIZING  SIM"

        cv2.putText(
            img,
            subtitle,
            (84, 112),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            LIGHT,
            1,
            cv2.LINE_AA
        )

        x = 54
        y = 128
        w = 212
        h = 14
        progress = min(1.0, max(0.0, t / max(self.boot_duration, 0.001)))

        cv2.rectangle(img, (x, y), (x + w, y + h), WHITE, 1)
        cv2.rectangle(img, (x + 2, y + 2), (x + w - 2, y + h - 2), (45, 45, 45), -1)

        fill_w = int((w - 4) * progress)
        if fill_w > 0:
            cv2.rectangle(img, (x + 2, y + 2), (x + 2 + fill_w, y + h - 2), GREEN, -1)

        cv2.putText(
            img,
            "BOOTING",
            (124, 156),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            LIGHT,
            1,
            cv2.LINE_AA
        )

        return img

    def render_mode_flash(self, mode: str, remaining: float) -> np.ndarray:
        width = 320
        height = 172
        img = np.zeros((height, width, 3), dtype=np.uint8)

        WHITE = (240, 240, 240)
        LIGHT = (185, 185, 185)

        bg_color = self.mode_flash_colors(mode)

        img[:] = bg_color

        # subtle pulse box
        pulse = 0.5 + 0.5 * math.sin(10.0 * (self.mode_flash_duration - remaining))
        alpha_box = int(20 + 30 * pulse)

        cv2.rectangle(img, (18, 18), (width - 19, height - 19), (alpha_box, alpha_box, alpha_box), -1)
        cv2.rectangle(img, (18, 18), (width - 19, height - 19), WHITE, 2)

        cv2.putText(
            img,
            self.robot_name[:10],
            (24, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            WHITE,
            1,
            cv2.LINE_AA
        )

        text_size = cv2.getTextSize(mode[:12], cv2.FONT_HERSHEY_SIMPLEX, 1.75, 3)[0]
        text_x = max(20, (width - text_size[0]) // 2)
        text_y = 96

        cv2.putText(
            img,
            mode[:12],
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.75,
            WHITE,
            3,
            cv2.LINE_AA
        )

        cv2.putText(
            img,
            "MODE CHANGE",
            (100, 126),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            LIGHT,
            1,
            cv2.LINE_AA
        )

        return img

    def render_dashboard(self, s: DisplayState) -> np.ndarray:
        width = 320
        height = 172
        img = np.zeros((height, width, 3), dtype=np.uint8)

        BG = (16, 16, 16)
        PANEL = (34, 34, 34)
        PANEL2 = (26, 26, 26)
        BORDER = (70, 70, 70)
        WHITE = (240, 240, 240)
        LIGHT = (175, 175, 175)
        GREEN = (60, 220, 90)
        YELLOW = (0, 220, 255)
        RED = (60, 60, 240)
        BLUE = (220, 170, 60)
        CYAN = (220, 220, 60)

        img[:] = BG
        cv2.rectangle(img, (2, 2), (width - 3, height - 3), BORDER, 1)

        # HEADER
        cv2.rectangle(img, (6, 6), (width - 7, 34), PANEL, -1)

        cv2.putText(
            img,
            self.robot_name[:10],
            (14, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            WHITE,
            2,
            cv2.LINE_AA
        )

        mode_color = self.mode_color(s.mode, GREEN, YELLOW, RED, BLUE)
        cv2.rectangle(img, (92, 9), (220, 31), mode_color, -1)

        cv2.putText(
            img,
            f"MODE: {s.mode[:10]}",
            (100, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (20, 20, 20),
            1,
            cv2.LINE_AA
        )

        self.draw_wifi_icon(img, 260, 10, s.wifi_bars, s.wifi_text, CYAN)

        # BATTERY PANEL
        cv2.rectangle(img, (6, 42), (width - 7, 92), PANEL2, -1)

        cv2.putText(
            img,
            "BATTERY",
            (14, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            LIGHT,
            1,
            cv2.LINE_AA
        )

        self.draw_battery_bar(
            img,
            x=14,
            y=66,
            w=220,
            h=18,
            percent=s.battery_percent
        )

        cv2.putText(
            img,
            f"{s.battery_percent:02d}%",
            (244, 72),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            WHITE,
            2,
            cv2.LINE_AA
        )

        cv2.putText(
            img,
            f"{s.battery_voltage:0.2f}V",
            (244, 88),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            LIGHT,
            1,
            cv2.LINE_AA
        )

        # HEALTH PANEL
        cv2.rectangle(img, (6, 100), (width - 7, 136), PANEL, -1)

        self.draw_health_chip(img, "IMU", "OK" if s.imu_ok else "LOST", 14, 108, 90, good=s.imu_ok)
        self.draw_health_chip(img, "ROS", "OK" if s.ros_ok else "DOWN", 112, 108, 90, good=s.ros_ok)

        temp_good = s.cpu_temp_c < 70.0 if s.cpu_temp_c > 0 else True
        temp_text = f"{s.cpu_temp_c:.0f}C" if s.cpu_temp_c > 0 else "--"
        self.draw_health_chip(img, "CPU", temp_text, 210, 108, 96, good=temp_good)

        # STATUS BAR
        cv2.rectangle(img, (6, 144), (width - 7, 166), PANEL2, -1)

        cv2.putText(
            img,
            f"STATUS: {s.status_text[:26]}",
            (12, 160),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            YELLOW,
            1,
            cv2.LINE_AA
        )

        return img

    def mode_color(self, mode, green, yellow, red, blue):
        m = mode.upper()
        if m in ["ERROR", "FAULT", "STOP"]:
            return red
        if m in ["LAY", "POSE", "IDLE"]:
            return yellow
        if m in ["STAND"]:
            return blue
        return green

    def mode_flash_colors(self, mode):
        m = mode.upper()
        if m in ["ERROR", "FAULT", "STOP"]:
            return (40, 40, 180)   # red-ish in BGR
        if m in ["LAY", "POSE", "IDLE"]:
            return (0, 170, 220)   # yellow-ish
        if m in ["STAND"]:
            return (170, 110, 40)  # blue-ish accent
        return (40, 140, 50)       # walk/default green-ish

    def draw_battery_bar(self, img, x, y, w, h, percent):
        WHITE = (240, 240, 240)
        GREEN = (60, 220, 90)
        YELLOW = (0, 220, 255)
        RED = (60, 60, 240)
        DARK = (55, 55, 55)

        cv2.rectangle(img, (x, y), (x + w, y + h), WHITE, 1)
        cv2.rectangle(img, (x + w + 3, y + h // 4), (x + w + 7, y + 3 * h // 4), WHITE, -1)

        inner_pad = 2
        inner_w = w - 2 * inner_pad
        fill_w = int(inner_w * max(0, min(100, percent)) / 100.0)

        if percent <= 20:
            color = RED
        elif percent <= 50:
            color = YELLOW
        else:
            color = GREEN

        cv2.rectangle(
            img,
            (x + inner_pad, y + inner_pad),
            (x + w - inner_pad, y + h - inner_pad),
            DARK,
            -1
        )

        if fill_w > 0:
            cv2.rectangle(
                img,
                (x + inner_pad, y + inner_pad),
                (x + inner_pad + fill_w, y + h - inner_pad),
                color,
                -1
            )

    def draw_health_chip(self, img, label, value, x, y, w, good=True):
        PANEL = (46, 46, 46)
        BORDER = (80, 80, 80)
        LIGHT = (180, 180, 180)
        WHITE = (240, 240, 240)
        GREEN = (60, 220, 90)
        RED = (60, 60, 240)
        YELLOW = (0, 220, 255)

        cv2.rectangle(img, (x, y), (x + w, y + 20), PANEL, -1)
        cv2.rectangle(img, (x, y), (x + w, y + 20), BORDER, 1)

        cv2.putText(
            img,
            label,
            (x + 6, y + 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            LIGHT,
            1,
            cv2.LINE_AA
        )

        if value in ["OK", "LOST", "DOWN"]:
            color = GREEN if good else RED
        else:
            color = WHITE if good else YELLOW

        cv2.putText(
            img,
            value,
            (x + 40, y + 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            color,
            1,
            cv2.LINE_AA
        )

    def draw_wifi_icon(self, img, x, y, bars, label, color):
        LIGHT = (140, 140, 140)
        WHITE = (240, 240, 240)

        bar_w = 6
        gap = 3
        base_y = y + 15
        heights = [4, 7, 10, 13]

        for i in range(4):
            bx = x + i * (bar_w + gap)
            by = base_y - heights[i]
            c = color if i < bars else LIGHT
            cv2.rectangle(img, (bx, by), (bx + bar_w, base_y), c, -1)

        cv2.putText(
            img,
            label[:8],
            (x - 6, y + 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.32,
            WHITE,
            1,
            cv2.LINE_AA
        )

    def destroy_node(self):
        self.backend.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JaxDisplayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
