"""
================================================================
plant_node.py  —  ROS 2 Node: Robot Plant Simulator  [FIXED]
JetBot AI Kit Waveshare  |  TA Motion Control
================================================================
PERBAIKAN [FIX-PLANT-N1]:
  MASALAH LAMA: saat tidak running (belum dapat /motor_voltage),
  plant_node tidak publish /robot_state sama sekali. Akibatnya
  kbbc_node tidak bisa mendapat state_ready=True, sehingga
  /system_ready tidak pernah dikirim ke trajectory_node.
  Ini membuat startup deadlock: semua node menunggu satu sama lain.

  SOLUSI: plant_node selalu publish /robot_state (posisi awal)
  bahkan saat belum running. Ini memungkinkan kbbc_node mendapat
  state_ready=True dan mengirim /system_ready ke trajectory.

PERBAIKAN [FIX-PLANT-N2]:
  MASALAH LAMA: plant_node langsung running=True begitu dapat
  /motor_voltage pertama (termasuk volt=0.0 dari pid_node).
  Ini menyebabkan robot_plant.step() dipanggil terus dengan
  volt=0, boros CPU.

  SOLUSI: plant_node running=True hanya jika |volt_L| atau
  |volt_R| > threshold kecil (0.01V). Ini mencegah step()
  dipanggil sia-sia saat volt=0.

SUBSCRIBE:
  /motor_voltage  [Float64MultiArray] — [volt_L, volt_R]
  /sim_control    [Int32]

PUBLISH:
  /robot_state         [Float64MultiArray] — [x,y,th,v,w]
  /motor_state         [Float64MultiArray] — [tau_R,tau_L,iR,iL,omR,omL]
  /wheel_speed_plant   [Float64MultiArray] — [omR, omL]
================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32

from jetbot_control.robot_plant import RobotPlant

# Threshold voltase — di bawah ini, plant dianggap diam
VOLT_THRESHOLD = 0.01   # V


class PlantNode(Node):

    def __init__(self):
        super().__init__('plant_node')

        self.declare_parameter('x0',  0.0)
        self.declare_parameter('y0',  0.0)
        self.declare_parameter('th0', 0.0)
        self.declare_parameter('dt',  0.01)

        x0  = self.get_parameter('x0').value
        y0  = self.get_parameter('y0').value
        th0 = self.get_parameter('th0').value
        self.dt = self.get_parameter('dt').value

        self.robot   = RobotPlant(x0, y0, th0)
        self.volt_L  = 0.0
        self.volt_R  = 0.0
        self.running = False

        # [FIX-PLANT-N1] Simpan posisi awal untuk publish saat idle
        self.x0  = x0; self.y0  = y0; self.th0 = th0

        # Publishers
        self.pub_state  = self.create_publisher(
            Float64MultiArray, '/robot_state', 10)
        self.pub_motor  = self.create_publisher(
            Float64MultiArray, '/motor_state', 10)
        self.pub_wheels = self.create_publisher(
            Float64MultiArray, '/wheel_speed_plant', 10)

        # Subscribers
        self.sub_volt = self.create_subscription(
            Float64MultiArray, '/motor_voltage', self._cb_volt, 10)
        self.sub_ctrl = self.create_subscription(
            Int32, '/sim_control', self._cb_control, 10)

        self.timer = self.create_timer(self.dt, self._timer_cb)

        self.get_logger().info(
            'plant_node siap [FIX-PLANT-N1,N2].\n'
            '  Posisi awal: x0=%.2f y0=%.2f th0=%.2f\n'
            '  Selalu publish /robot_state (termasuk saat idle)\n'
            '  running=True hanya saat |volt|>%.3fV' %
            (x0, y0, th0, VOLT_THRESHOLD))

    def _cb_volt(self, msg):
        if len(msg.data) >= 2:
            self.volt_L = msg.data[0]
            self.volt_R = msg.data[1]
            # [FIX-PLANT-N2] Hanya running jika ada voltase nyata
            if abs(self.volt_L) > VOLT_THRESHOLD or abs(self.volt_R) > VOLT_THRESHOLD:
                self.running = True

    def _cb_control(self, msg):
        cmd = msg.data
        if cmd == 9:
            x0  = self.get_parameter('x0').value
            y0  = self.get_parameter('y0').value
            th0 = self.get_parameter('th0').value
            self.robot.reset(x0, y0, th0)
            self.volt_L  = 0.0
            self.volt_R  = 0.0
            self.running = False
            self.get_logger().info('RESET — posisi robot kembali ke awal')
        elif cmd == 99:
            self.running = False
            self.get_logger().info('STOP diterima')

    def _timer_cb(self):
        if self.running:
            # Jalankan fisika Lagrangian (RK4 internal, SIM_DT=1ms)
            v, w, x, y, th, tau_R, tau_L = self.robot.step(
                self.volt_L, self.volt_R, self.dt)
            omR, omL = self.robot.wheel_speeds
        else:
            # [FIX-PLANT-N1] Saat idle: publish posisi saat ini (awal)
            x, y, th = self.robot.pos
            v = 0.0; w = 0.0
            tau_R = 0.0; tau_L = 0.0
            omR = 0.0; omL = 0.0

        # Selalu publish robot_state agar kbbc_node bisa state_ready=True
        msg_state = Float64MultiArray()
        msg_state.data = [x, y, th, v, w]
        self.pub_state.publish(msg_state)

        # Hanya publish motor_state dan wheel_speed saat running
        if self.running:
            msg_motor = Float64MultiArray()
            msg_motor.data = [
                tau_R, tau_L,
                self.robot.i_R, self.robot.i_L,
                omR, omL
            ]
            self.pub_motor.publish(msg_motor)

            msg_wheels = Float64MultiArray()
            msg_wheels.data = [omR, omL]
            self.pub_wheels.publish(msg_wheels)


def main(args=None):
    rclpy.init(args=args)
    node = PlantNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
