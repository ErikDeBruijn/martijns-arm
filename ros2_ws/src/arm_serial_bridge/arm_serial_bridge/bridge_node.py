"""ROS2 brug van v18.1/v19 firmware ASCII-stream naar joint_states + diagnostics.

Verbinding loopt via TCP (socat draait op de Mac, host.docker.internal:9999),
zodat we Docker Desktop's gebrek aan USB-passthrough omzeilen.
"""

import math
import re
import socket
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String

# Format v18.1+:
# PB t=4105 sc=1.00  M1 ref=-65.4° enc=-65.5° err=0.16° spd=-6652 cmd=-5%   M2 ... M3 ...
PB_LINE = re.compile(r"^PB\s+t=(\d+)\s+sc=([\d\.\-]+)\s+(.*)$")
JOINT_BLOCK = re.compile(
    r"M(?P<idx>[123])\s+"
    r"ref=(?P<ref>[\-\d\.]+)°?\s+"
    r"enc=(?P<enc>[\-\d\.]+)°?\s+"
    r"err=(?P<err>[\-\d\.]+)°?\s+"
    r"spd=(?P<spd>[\-\d\.]+)\s+"
    r"cmd=(?P<cmd>[\-\d\.\+]+)%"
)
JOINT_NAMES = ["m1", "m2", "m3"]


class ArmSerialBridge(Node):
    def __init__(self, host: str, port: int):
        super().__init__("arm_serial_bridge")
        self.host = host
        self.port = port

        self.pub_joints = self.create_publisher(JointState, "/joint_states", 50)
        self.pub_scale = self.create_publisher(Float32, "/arm/time_scale", 10)
        self.pub_response = self.create_publisher(String, "/arm/response", 50)
        # Per-joint Float32 topics — handig voor drag-drop plotten in Foxglove
        self.pub_per_joint = {
            j: {
                field: self.create_publisher(Float32, f"/arm/{j}/{field}", 50)
                for field in ("ref_deg", "enc_deg", "err_deg", "spd_steps_s", "cmd_pct")
            }
            for j in JOINT_NAMES
        }

        self.sock: Optional[socket.socket] = None
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def _connect(self) -> socket.socket:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.host, self.port))
        s.settimeout(5.0)
        self.get_logger().info(f"Connected to {self.host}:{self.port}")
        return s

    def _reader_loop(self):
        buf = b""
        backoff = 1.0
        while rclpy.ok():
            try:
                if self.sock is None:
                    self.sock = self._connect()
                    backoff = 1.0
                try:
                    chunk = self.sock.recv(4096)
                except socket.timeout:
                    continue   # IDLE arm = no PB output, dat is normaal
                if not chunk:
                    raise ConnectionError("EOF from socat")
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self._handle_line(line.decode("utf-8", errors="replace").strip())
            except (ConnectionError, OSError) as exc:
                self.get_logger().warn(f"Connection issue: {exc}; retry in {backoff:.1f}s")
                if self.sock:
                    try: self.sock.close()
                    except Exception: pass
                self.sock = None
                rclpy.spin_once(self, timeout_sec=backoff)
                backoff = min(backoff * 2, 10.0)

    def _handle_line(self, line: str):
        if not line:
            return
        if line.startswith("<"):
            msg = String(); msg.data = line
            self.pub_response.publish(msg)
            return
        m = PB_LINE.match(line)
        if not m:
            return  # skip non-PB output (banner, info messages, etc.)

        time_scale = float(m.group(2))
        rest = m.group(3)
        joints = {jb.group("idx"): jb for jb in JOINT_BLOCK.finditer(rest)}
        if len(joints) != 3:
            return  # malformed line, skip silently

        scale_msg = Float32(); scale_msg.data = time_scale
        self.pub_scale.publish(scale_msg)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name, js.position, js.effort = [], [], []
        for i, name in enumerate(JOINT_NAMES, start=1):
            jb = joints[str(i)]
            ref = float(jb.group("ref"))
            enc = float(jb.group("enc"))
            err = float(jb.group("err"))
            spd = float(jb.group("spd"))
            cmd = float(jb.group("cmd").lstrip("+"))
            js.name.append(name)
            js.position.append(math.radians(enc))   # arm-side angle in rad
            js.effort.append(cmd)                    # commanded effort %
            for field, val in (
                ("ref_deg", ref), ("enc_deg", enc), ("err_deg", err),
                ("spd_steps_s", spd), ("cmd_pct", cmd),
            ):
                fmsg = Float32(); fmsg.data = val
                self.pub_per_joint[name][field].publish(fmsg)
        self.pub_joints.publish(js)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="host.docker.internal")
    parser.add_argument("--port", type=int, default=9999)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = ArmSerialBridge(args.host, args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
