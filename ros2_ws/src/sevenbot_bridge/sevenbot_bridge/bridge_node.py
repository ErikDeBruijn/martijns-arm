"""ROS2 brug voor de 7Bot robotarm.

Wrapt de SevenBot Python library (van dehavenm/7bot-Python, BSD) en stelt
hem bloot als ROS2 topics + services. Werkt met dezelfde JointTrajectory
interface als ons Gazebo en Martijns-arm setups, dus controllers en
trainers werken op alle drie identiek.

Topics:
    /joint_states                       sensor_msgs/JointState   (publish, 50Hz)
    /sevenbot/joint_trajectory          trajectory_msgs/JointTrajectory  (subscribe)

Services:
    /sevenbot/set_force_status          std_srvs/SetBool   (true=normal, false=forceless)
    /sevenbot/set_vacuum                std_srvs/SetBool   (true=aan, false=uit)
    /sevenbot/set_gripper               std_srvs/Trigger   (toggle, of via joint_6 angle)

Joint mapping:
    joint_0 .. joint_5 → servo 0 .. 5 (in radialen, intern naar graden gemapt)
    Servo 6 = gripper of vacuüm:
       - vacuüm: angle=0 → AAN, angle=180 → UIT
       - gripper: 0..pi/2

Argumenten:
    --port      USB serial poort (default /dev/cu.usbserial-* eerste match)
    --baud      baud rate (default 115200)
    --rate      publish rate in Hz (default 50)
"""

import argparse
import glob
import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

from .sevenbot import SevenBot

JOINT_NAMES = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
INITIAL_ANGLES_DEG = [90.0, 115.0, 65.0, 90.0, 90.0, 90.0]   # uit Arm7Bot.h


def detect_default_port() -> Optional[str]:
    """Vind de Arduino Due USB serial automatisch op macOS/Linux."""
    candidates = (
        glob.glob("/dev/cu.usbserial-*")
        + glob.glob("/dev/cu.usbmodem*")
        + glob.glob("/dev/ttyACM*")
        + glob.glob("/dev/ttyUSB*")
    )
    return candidates[0] if candidates else None


class SevenBotBridge(Node):
    def __init__(self, port: str, baud: int, rate: float):
        super().__init__("sevenbot_bridge")
        self.get_logger().info(f"Open serial: {port} @ {baud}")
        self.arm = SevenBot(port, baud)
        self.gripper_angle = 75.0   # default uit firmware

        # Publishers / subscribers
        self.pub_joints = self.create_publisher(JointState, "/joint_states", 50)
        self.create_subscription(
            JointTrajectory, "/sevenbot/joint_trajectory", self._on_trajectory, 10
        )

        # Services
        self.create_service(SetBool, "/sevenbot/set_force_status", self._srv_force)
        self.create_service(SetBool, "/sevenbot/set_vacuum", self._srv_vacuum)
        self.create_service(Trigger, "/sevenbot/set_gripper", self._srv_gripper_toggle)

        # 50Hz publish loop
        self.create_timer(1.0 / rate, self._publish_state)
        self.get_logger().info("Klaar. /joint_states publiceert; /sevenbot/joint_trajectory accepteert commando's.")

    # ─── command path ────────────────────────────────────────────
    def _on_trajectory(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn("Lege trajectory ontvangen, genegeerd.")
            return
        # Voor nu: spring naar laatste punt; tijden negeren we tijdelijk
        target = msg.points[-1].positions
        if len(target) != len(JOINT_NAMES):
            self.get_logger().error(f"Verwacht {len(JOINT_NAMES)} joints, kreeg {len(target)}")
            return

        # rad → graden, plus gripper als 7e element
        angles_deg = [math.degrees(p) for p in target] + [self.gripper_angle]
        angles_arr = np.array(angles_deg, dtype=np.float32)
        self.arm.setAngle(angles_arr)
        self.get_logger().debug(f"setAngle {angles_deg}")

    # ─── state path ──────────────────────────────────────────────
    def _publish_state(self):
        positions_deg = self.arm._pos[:6]   # eerste 6 servo's
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [math.radians(float(p)) for p in positions_deg]
        self.pub_joints.publish(msg)

    # ─── services ────────────────────────────────────────────────
    def _srv_force(self, req: SetBool.Request, resp: SetBool.Response):
        self.arm.setForceStatus(1 if req.data else 0)
        resp.success = True
        resp.message = "force=normal" if req.data else "force=off (handmatig bewegen ok)"
        return resp

    def _srv_vacuum(self, req: SetBool.Request, resp: SetBool.Response):
        # vacuüm zit op servo 6, 0=aan, 180=uit
        angle = 0.0 if req.data else 180.0
        cmd = np.array(list(self.arm.angle[:6]) + [angle], dtype=np.float32)
        self.arm.setAngle(cmd)
        self.gripper_angle = angle
        resp.success = True
        resp.message = f"vacuüm {'aan' if req.data else 'uit'}"
        return resp

    def _srv_gripper_toggle(self, req: Trigger.Request, resp: Trigger.Response):
        new = 0.0 if self.gripper_angle > 45.0 else 90.0
        self.gripper_angle = new
        cmd = np.array(list(self.arm.angle[:6]) + [new], dtype=np.float32)
        self.arm.setAngle(cmd)
        resp.success = True
        resp.message = f"gripper -> {new:.0f}°"
        return resp


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=detect_default_port())
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=50.0)
    args, _ = parser.parse_known_args()

    if not args.port:
        print("ERROR: geen USB serial poort gevonden. Geef --port mee.")
        return 1

    rclpy.init()
    node = SevenBotBridge(args.port, args.baud, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
