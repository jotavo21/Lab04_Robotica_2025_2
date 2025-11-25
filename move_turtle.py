import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import curses
import time

class TurtleTeleop(Node):
    def __init__(self, screen):
        super().__init__("turtle_teleop_letters_reset")

        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.reset_cli = self.create_client(Empty, "/reset")

        while not self.reset_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Esperando servicio /reset ...")

        self.linear_speed = 1.5
        self.angular_speed = 1.45

        self.screen = screen
        self.screen.nodelay(True)
        curses.cbreak()

        self.timer = self.create_timer(0.1, self.read_keyboard)

        self.get_logger().info("Teleop listo. Flechas = mover. Letras = dibujar con reset (I S R P J E O G).")

    # ----------------------------
    # RESET
    # ----------------------------
    def reset_turtle(self):
        req = Empty.Request()
        self.reset_cli.call_async(req)
        time.sleep(0.2)

    # ----------------------------
    # MOVIMIENTOS PRIMITIVOS
    # ----------------------------
    def move_forward(self, t):
        msg = Twist()
        msg.linear.x = self.linear_speed
        start = time.time()
        while time.time() - start < t:
            self.pub.publish(msg)
            time.sleep(0.01)
        self.stop()

    def move_backward(self, t):
        msg = Twist()
        msg.linear.x = -self.linear_speed
        start = time.time()
        while time.time() - start < t:
            self.pub.publish(msg)
            time.sleep(0.01)
        self.stop()

    def turn_left(self, t):
        msg = Twist()
        msg.angular.z = self.angular_speed
        start = time.time()
        while time.time() - start < t:
            self.pub.publish(msg)
            time.sleep(0.01)
        self.stop()

    def turn_right(self, t):
        msg = Twist()
        msg.angular.z = -self.angular_speed
        start = time.time()
        while time.time() - start < t:
            self.pub.publish(msg)
            time.sleep(0.01)
        self.stop()

    def stop(self):
        msg = Twist()
        self.pub.publish(msg)

    # ----------------------------
    # LETRAS SOLO CON LÍNEAS RECTAS
    # ----------------------------

    # todas las letras empiezan igual: reset + un punto base

    def draw_I(self):
        self.get_logger().info("Dibujando I")
        self.reset_turtle()
        self.turn_left(1)
        self.move_forward(2)

    def draw_S(self):
        self.get_logger().info("Dibujando S")
        self.reset_turtle()

        self.move_forward(1.5)         # ─
        self.turn_left(1)              # │
        self.move_forward(1.5)
        self.turn_left(1)              # ─
        self.move_forward(1.5)
        self.turn_right(1)             # │
        self.move_forward(1.5)
        self.turn_right(1)
        self.move_forward(1.5)

    def draw_R(self):
        self.get_logger().info("Dibujando R")
        self.reset_turtle()

        self.turn_left(1)
        self.move_forward(3)           # palo vertical
        self.turn_right(1)
        self.move_forward(1.5)         # parte superior
        self.turn_right(1)
        self.move_forward(1.5)         # baja mitad círculo
        self.turn_right(1)
        self.move_forward(1.5)         # vuelve al palo
        self.turn_left(1)
        self.move_forward(1.5)         # patita diagonal

    def draw_P(self):
        self.get_logger().info("Dibujando P")
        self.reset_turtle()

        self.turn_left(1)
        self.move_forward(3)
        self.turn_right(1)
        self.move_forward(1.5)
        self.turn_right(1)
        self.move_forward(1.5)
        self.turn_right(1)
        self.move_forward(1.5)

    def draw_J(self):
        self.get_logger().info("Dibujando J")
        self.reset_turtle()

        self.turn_left(1)
        self.turn_right(1)
        self.move_forward(2)
        self.turn_left(1)
        self.move_forward(3)
        self.turn_left(1)
        self.move_forward(1)

    def draw_E(self):
        self.get_logger().info("Dibujando E")
        self.reset_turtle()

        self.turn_left(1)
        self.move_forward(3)
        self.turn_right(1)
        self.move_forward(1.5)
        self.turn_backward = 1
        self.turn_left(1)
        self.move_backward(1.5)
        self.turn_left(1)
        self.move_forward(1.5)
        self.turn_right(1)
        self.move_forward(1)

    def draw_O(self):
        self.get_logger().info("Dibujando O")
        self.reset_turtle()
        self.turn_left(1)

        for _ in range(4):
            self.move_forward(2)
            self.turn_left(1)

    def draw_G(self):
        self.get_logger().info("Dibujando G")
        self.reset_turtle()

        self.turn_left(1)
        for _ in range(3):
            self.move_forward(2)
            self.turn_left(1)
        self.move_forward(2)
        self.turn_right(1)
        self.move_forward(1)

    # ----------------------------
    # LECTURA DE TECLADO
    # ----------------------------
    def read_keyboard(self):
        key = self.screen.getch()
        msg = Twist()

        # Movimiento normal
        if key == curses.KEY_UP:
            msg.linear.x = self.linear_speed
        elif key == curses.KEY_DOWN:
            msg.linear.x = -self.linear_speed
        elif key == curses.KEY_LEFT:
            msg.angular.z = self.angular_speed
        elif key == curses.KEY_RIGHT:
            msg.angular.z = -self.angular_speed
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.pub.publish(msg)

        # Letras con reset
        if key in [ord("I"), ord("i")]: self.draw_I()
        if key in [ord("S"), ord("s")]: self.draw_S()
        if key in [ord("R"), ord("r")]: self.draw_R()
        if key in [ord("P"), ord("p")]: self.draw_P()
        if key in [ord("J"), ord("j")]: self.draw_J()
        if key in [ord("E"), ord("e")]: self.draw_E()
        if key in [ord("O"), ord("o")]: self.draw_O()
        if key in [ord("G"), ord("g")]: self.draw_G()


def main():
    rclpy.init()
    screen = curses.initscr()
    curses.noecho()
    screen.keypad(True)

    try:
        node = TurtleTeleop(screen)
        rclpy.spin(node)
    finally:
        curses.nocbreak()
        screen.keypad(False)
        curses.echo()
        curses.endwin()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
