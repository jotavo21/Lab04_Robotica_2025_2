import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


import sys, termios, tty, time





class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_turtle)

    def move_turtle(self):
        key = get_key_non_blocking()
        # Si una tecla fue presionada, actualiza la última tecla presionada
        if key is not None:
            self.last_key = key
        
        if key == 'c': 
            msg = Twist()
            msg.linear.x = 2.0  # Velocidad hacia adelante
            msg.angular.z = 0.0  # Rotación
            start_time = time.time()
            while time.time() - start_time < 3.0:
                self.publisher_.publish(msg)
                self.get_logger().info('Moviendo la tortuga')
            msg.linear.x = 0.0  # Velocidad hacia adelante
            msg.angular.z = 0.0  # Rotación
def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def get_key():
    """Lee una sola tecla sin necesidad de presionar Enter"""
    fd = sys.stdin.fileno()         # descriptor del teclado
    old_settings = termios.tcgetattr(fd)  # guarda configuración actual
    try:
        tty.setraw(fd)              # modo raw (sin buffering)
        key = sys.stdin.read(1)     # lee un solo carácter
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # restaura configuración
    return key


def get_key_non_blocking():
    """Lee hasta 3 caracteres (para flechas) si están disponibles, sino devuelve None."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    try:
        tty.setraw(fd)
        # Verifica si hay datos en stdin (la tecla)
        if select.select([sys.stdin], [], [], 0.0)[0]:
            # Intenta leer hasta 3 caracteres para capturar secuencias de flechas (ej. '\x1b[A')
            key = sys.stdin.read(3) 
            return key
        else:
            return None # No hay tecla presionada
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)