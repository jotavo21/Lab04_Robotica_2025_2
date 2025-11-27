import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import curses
import time


class TurtleTeleop(Node):
    def __init__(self, screen):
        super().__init__("turtle_controller")

        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.reset_cli = self.create_client(Empty, "/reset")
        self.linear_speed = 1.5
        self.angular_speed = 1.45
        self.screen = screen
        self.screen.nodelay(True)
        curses.cbreak()

        # Timer que lee el teclado periódicamente
        self.timer = self.create_timer(0.1, self.read_keyboard)

        self.get_logger().info(
            "Moviendo la tortuga: Flechas verticales = Avanzar o Retroceder. Flechas horizontales = Girar. Letras = Dibujar la letra (I S R P J E O G)."
        )

    # ----------------------------
    # UTILIDADES DE TECLADO
    # ----------------------------
    def _drain_keys(self):
        #Elimina las teclas que se acumulan la cola de entradas de teclado (FUnciona con todas las teclas)
        while True:
            ch = self.screen.getch()
            if ch == -1:
                break

    def _get_last_key(self):
        
        #Devuelve solo la última tecla de la cola de entradas de teclado, para evitar acumulacion (Principalmente para corregir errores de las entradas por flechas)
        last = -1
        while True:
            ch = self.screen.getch()
            if ch == -1:
                break
            last = ch
        return last

 
    #Resetear la posicion de la tortuga
    def reset_turtle(self):
        req = Empty.Request()
        self.reset_cli.call_async(req)
        time.sleep(0.2)


    # Avance, retroceso, giro horario y antihorario con la velocidad establecida por el tiempo determinado
    def move_forward(self, t):
        msg = Twist()
        msg.linear.x = self.linear_speed
        start = time.time()
        while time.time() - start < t:
            self.pub.publish(msg)
            self._drain_keys() 
            time.sleep(0.01)
        self.stop()

    def move_backward(self, t):
        msg = Twist()
        msg.linear.x = -self.linear_speed
        start = time.time()
        while time.time() - start < t:
            self.pub.publish(msg)
            self._drain_keys()
            time.sleep(0.01)
        self.stop()

    def turn_left(self, t):
        msg = Twist()
        msg.angular.z = self.angular_speed
        start = time.time()
        while time.time() - start < t:
            self.pub.publish(msg)
            self._drain_keys()
            time.sleep(0.01)
        self.stop()

    def turn_right(self, t):
        msg = Twist()
        msg.angular.z = -self.angular_speed
        start = time.time()
        while time.time() - start < t:
            self.pub.publish(msg)
            self._drain_keys()
            time.sleep(0.01)
        self.stop()

    def stop(self):
        msg = Twist()
        self.pub.publish(msg)


    #Dibujo de las iniciales de los integrantes

    def draw_I(self):
        self.get_logger().info("Dibujando I")
        self._drain_keys()        
        self.reset_turtle()
        self.turn_left(1)
        self.move_forward(2)

    def draw_S(self):
        self.get_logger().info("Dibujando S")
        self._drain_keys()
        self.reset_turtle()

        self.move_forward(1.5)         
        self.turn_left(1)              
        self.move_forward(1.5)
        self.turn_left(1)              
        self.move_forward(1.5)
        self.turn_right(1)             
        self.move_forward(1.5)
        self.turn_right(1)
        self.move_forward(1.5)

    def draw_R(self):
        self.get_logger().info("Dibujando R")
        self._drain_keys()
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
        self._drain_keys()
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
        self._drain_keys()
        self.reset_turtle()

        self.move_forward(1)
        self.turn_left(1.1)
        self.move_forward(2)
        self.turn_left(1.2)
        self.move_forward(0.8)
        self.move_backward(1.6)

    def draw_E(self):
        self.get_logger().info("Dibujando E")
        self._drain_keys()
        self.reset_turtle()

        self.turn_left(1.05)
        self.move_forward(3)      
        self.turn_right(1.03)
        
        self.move_forward(1.5)    
        self.move_backward(1.5)
        self.turn_left(1.03)
        for _ in range(2):
            self.move_backward(1.5)    
            self.move_forward(1.5)      
            self.move_backward(1.5)
            self.turn_left(1.03)

    def draw_O(self):
        self.get_logger().info("Dibujando O")
        self._drain_keys()
        self.reset_turtle()
        self.turn_left(1)
        msg = Twist()
        msg.angular.z = 1.0
        for _ in range(4):
            self.move_forward(1.8)
    def draw_G(self):
        self.get_logger().info("Dibujando G")
        self._drain_keys()
        self.reset_turtle()
        self.turn_left(1)
        self.move_forward(3)
        self.turn_right(1)
        self.move_forward(1.5)
        self.move_backward(1.5)
        self.turn_left(1)
        self.move_backward(3)
        self.turn_right(1)
        self.move_forward(1.5)

        for _ in range(2):
            self.turn_left(1)
            self.move_forward(1)
        self.turn_left(1.23)
        self.move_forward(0.5)


    
    #Lectura de teclado

    def read_keyboard(self):
        # Leer solo la última tecla pulsada desde el último ciclo
        key = self._get_last_key()
        msg = Twist()

        # Movimiento normal con flechas
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

        # Dibujo de letras
        if key in [ord("I"), ord("i")]:
            self.draw_I()
        if key in [ord("S"), ord("s")]:
            self.draw_S()
        if key in [ord("R"), ord("r")]:
            self.draw_R()
        if key in [ord("P"), ord("p")]:
            self.draw_P()
        if key in [ord("J"), ord("j")]:
            self.draw_J()
        if key in [ord("E"), ord("e")]:
            self.draw_E()
        if key in [ord("O"), ord("o")]:
            self.draw_O()
        if key in [ord("G"), ord("g")]:
            self.draw_G()


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
