import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportRelative, TeleportAbsolute, SetPen
import threading
import curses
import time
import math

# Clase principal que controla la tortuga en ROS2
class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        # Publicador para enviar comandos de movimiento a la tortuga
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Temporizador para actualizar la posición y controlar la tortuga
        self.timer = self.create_timer(0.3, self.move_turtle2)
        # Variables para controlar el estado
        self.running = True
        self.action = ""
        self.drawing = False
        self.turn_off = False

        # Suscripción para recibir la posición de la tortuga
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.x = 0.0
        self.y = 0.0

    def pose_callback(self, msg):
        # Callback para actualizar las coordenadas de la tortuga
        self.x = msg.x
        self.y = msg.y

    def clear_screen(self):
        # Limpia la pantalla de la simulación de la tortuga
        client = self.create_client(Empty, '/clear')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /clear...')

        req = Empty.Request()
        client.call_async(req)
        self.drawing = False

    def set_abs_angle(self, theta):
        # Establece el ángulo absoluto de la tortuga
        cliente = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not cliente.wait_for_service(timeout_sec=1.0):
            pass
        req = TeleportAbsolute.Request()
        req.x = float(self.x)
        req.y = float(self.y)
        req.theta = float(theta)
        cliente.call(req)
        time.sleep(0.2)

    def set_pen(self, pen_on=True):
        # Establece el estado del lápiz (encendido o apagado)
        cliente = self.create_client(SetPen, '/turtle1/set_pen')
        while not cliente.wait_for_service(timeout_sec=1.0):
            pass
        req = SetPen.Request()
        req.r = 255
        req.g = 255
        req.b = 255
        req.width = 2
        req.off = 0 if pen_on else 1
        cliente.call(req)

    def move_turtle(self):
        # Mueve la tortuga hacia adelante con velocidad y rotación
        msg = Twist()
        msg.linear.x = 2.0  # Velocidad hacia adelante
        msg.angular.z = 1.0  # Rotación
        self.publisher_.publish(msg)
        self.get_logger().info('Moviendo la tortuga')

    def move_turtle2(self):
        # Controla el movimiento basado en las teclas presionadas
        msg = Twist()

        if self.turn_off:
            self.running = False
            self.turn_off = False

        if self.drawing:
            threading.Thread(target=self.set_pen, args=(True,), daemon=True).start()

        if self.running or self.drawing:
            if self.action == "Up":
                msg.linear.x = 4.0
            elif self.action == "Down":
                msg.linear.x = -4.0
            elif self.action == "Left":
                threading.Thread(target=self.rotate_turtle, args=(math.radians(90),)).start()
            elif self.action == "Right":
                threading.Thread(target=self.rotate_turtle, args=(math.radians(-90),)).start()
            elif self.action == "Draw_M":
                threading.Thread(target=self.drawCharacter, args=('m',)).start()
            elif self.action == "Draw_A":
                threading.Thread(target=self.drawCharacter, args=('a',)).start()
            elif self.action == "Draw_S":
                threading.Thread(target=self.drawCharacter, args=('s',)).start()
            elif self.action == "Draw_I":
                threading.Thread(target=self.drawCharacter, args=('i',)).start()
            elif self.action == "Draw_C":
                threading.Thread(target=self.drawCharacter, args=('c',)).start()
            elif self.action == "Draw_P":
                threading.Thread(target=self.drawCharacter, args=('p',)).start()
            elif self.action == "Draw_V":
                self.clear_screen()

        if self.running:
            threading.Thread(target=self.set_pen, args=(False,), daemon=True).start()
            self.turn_off = True
            self.publisher_.publish(msg)
        elif not self.drawing:
            msg.linear.x = 0.0
            self.publisher_.publish(msg)

        self.action = ""

    def move_forward(self, speed, t):
        # Mueve la tortuga hacia adelante por un tiempo determinado
        msg = Twist()
        msg.linear.x = speed
        self.publisher_.publish(msg)
        time.sleep(t)
        msg.linear.x = 0.0
        self.publisher_.publish(msg)

    def drawCharacter(self, char):
        # Dibuja caracteres (M, S, I, A, C, P, V) con la tortuga
        if char == 'p':
            msg = Twist()

            self.set_abs_angle(math.pi/2)
            self.move_forward(2.5, 2.0)

            self.set_abs_angle(0)
            self.move_forward(2.0, 0.25)

            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            time.sleep(0.2)

            start_time = time.time()
            while time.time() - start_time < (math.radians(160))/4:
                msg.linear.x = 3.0
                msg.angular.z = -4.0
                self.publisher_.publish(msg)

            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            time.sleep(0.2)

            self.set_abs_angle(math.radians(180))
            self.move_forward(2.0, 0.2)

            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)

            self.set_abs_angle(math.radians(180))

        elif char == 'c':
            msg = Twist()

            self.set_abs_angle(math.radians(-(90+30)))

            start_time = time.time()
            while time.time() - start_time < ((3*math.pi/2)/4):
                msg.linear.x = 4.0
                msg.angular.z = -4.0
                self.publisher_.publish(msg)

            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            time.sleep(0.3)

            self.set_abs_angle(math.pi/2)

        elif char == 's':
            msg = Twist()

            self.set_abs_angle(math.pi/2)

            self.rotate_turtle(math.radians(180))
            time.sleep(0.3)
            
            start_time = time.time()
            while time.time() - start_time < ((3*math.pi/2)/4):
                msg.linear.x = 2.5
                msg.angular.z = 4.0
                self.publisher_.publish(msg)

            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)

            start_time = time.time()
            while time.time() - start_time < ((3*math.pi/2)/4):
                msg.linear.x = 2.5
                msg.angular.z = -4.0
                self.publisher_.publish(msg)

            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            time.sleep(0.1)

            self.set_abs_angle(math.pi/2)

        elif char == 'i':
            self.set_abs_angle(math.pi/2)

            self.rotate_turtle(math.radians(90))
            time.sleep(0.3)

            self.move_forward(4.0, 0.2)

            self.rotate_turtle(math.radians(-180))
            time.sleep(0.3)

            self.move_forward(4.0, 0.4)

            self.rotate_turtle(math.radians(180))
            time.sleep(0.3)

            self.move_forward(4.0, 0.2)

            self.rotate_turtle(math.radians(-90))
            time.sleep(0.3)

            self.move_forward(4.0, 0.5)

            self.rotate_turtle(math.radians(90))
            time.sleep(0.3)

            self.move_forward(4.0, 0.2)

            self.rotate_turtle(math.radians(-180))
            time.sleep(0.3)

            self.move_forward(4.0, 0.4)

            self.rotate_turtle(math.radians(90))
            time.sleep(0.3)

        elif char == 'm':
            self.set_abs_angle(math.pi/2)

            self.move_forward(4.0, 0.5)

            self.rotate_turtle(math.radians(135))
            time.sleep(0.3)

            self.move_forward(4.0, 0.2)

            self.rotate_turtle(math.radians(-90))
            time.sleep(0.3)

            self.move_forward(4.0, 0.2)

            self.rotate_turtle(math.radians(135))
            time.sleep(0.3)

            self.move_forward(4.0, 0.5)

        elif char == 'a':
            self.set_abs_angle(math.pi/2)

            self.rotate_turtle(math.radians(20))
            time.sleep(0.3)

            self.move_forward(4.0, 0.6)

            self.rotate_turtle(math.radians(140))
            time.sleep(0.3)

            self.move_forward(4.0, 0.3)

            self.rotate_turtle(math.radians(110))
            time.sleep(0.3)

            self.move_forward(4.0, 0.2)

            self.rotate_turtle(math.radians(-180))
            time.sleep(0.3)

            self.move_forward(4.0, 0.2)

            self.rotate_turtle(math.radians(70))
            time.sleep(0.3)

            self.move_forward(4.0, 0.3)

            self.rotate_turtle(math.radians(20))
            time.sleep(0.3)

        self.drawing = False

    def rotate_turtle(self, angulo):
        # Rota la tortuga en un ángulo relativo
        cliente = self.create_client(TeleportRelative, '/turtle1/teleport_relative')

        while not cliente.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio teleport_relative...')

        req = TeleportRelative.Request()
        req.linear = 0.0
        req.angular = float(angulo)

        cliente.call(req)
        time.sleep(0.2)

    def read_keys(self, stdscr, stop_event):
        # Lee las teclas de control para mover la tortuga
        curses.curs_set(0)  # Ocultar el cursor
        stdscr.nodelay(1)  # Evitar bloqueo esperando tecla

        while rclpy.ok() and not stop_event.is_set():
            stdscr.clear()
            stdscr.addstr(0, 0, "Controles:")
            stdscr.addstr(1, 0, "↑ - Adelante | ↓ - Atras | ← - Giro Izquierda | → - Giro Derecha")
            stdscr.addstr(2, 0, "[M ó m] - Dibujar una M")
            stdscr.addstr(3, 0, "[S ó s] - Dibujar una S")
            stdscr.addstr(4, 0, "[I ó i] - Dibujar una I")
            stdscr.addstr(5, 0, "[A ó a] - Dibujar una A")
            stdscr.addstr(6, 0, "[C ó c] - Dibujar una C")
            stdscr.addstr(7, 0, "[V ó v] - Limpiar la pantalla")

            estado = "Dibujando" if self.drawing else ("Moviendo" if self.running else "Detenido")
            stdscr.addstr(9, 0, f"Estado actual: {estado}")
            stdscr.addstr(10, 0, f"X: {round(self.x, 1)}, Y: {round(self.y, 1)}")
            if self.action == "":
                key = -1
                while True:
                    k = stdscr.getch()
                    if k == -1:
                        break
                    key = k

                if key != -1:
                    stdscr.addstr(11, 0, f"Key: {chr(key)}")
                    try:
                        # Verifica la tecla presionada y realiza la acción correspondiente
                        if chr(key).lower() == 'm':
                            self.drawing = True
                            self.action = "Draw_M"
                        elif chr(key).lower() == 's':
                            self.drawing = True
                            self.action = "Draw_S"
                        elif chr(key).lower() == 'i':
                            self.drawing = True
                            self.action = "Draw_I"
                        elif chr(key).lower() == 'a':
                            self.drawing = True
                            self.action = "Draw_A"
                        elif chr(key).lower() == 'c':
                            self.drawing = True
                            self.action = "Draw_C"
                        elif chr(key).lower() == 'v':
                            self.drawing = False
                            self.action = "Draw_V"
                        elif chr(key).lower() == 'w':
                            self.action = "Up"
                        elif chr(key).lower() == 's':
                            self.action = "Down"
                        elif chr(key).lower() == 'a':
                            self.action = "Left"
                        elif chr(key).lower() == 'd':
                            self.action = "Right"
                        elif chr(key).lower() == 'q':
                            self.running = False
                            stop_event.set()
                            break
                    except Exception as e:
                        print(e)

            time.sleep(0.1)

    def main():
        # Inicia ROS2 y crea el nodo
        rclpy.init()
        controller = TurtleController()
    
        stop_event = threading.Event()
        stdscr_thread = threading.Thread(target=controller.read_keys, args=(None, stop_event), daemon=True)
    
        stdscr_thread.start()
        rclpy.spin(controller)
    
        controller.destroy_node()
        rclpy.shutdown()
    
    if __name__ == '__main__':
        main()
