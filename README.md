# Laboratorio No. 04 - 2025-II - Robótica de Desarrollo, Intro a ROS 2 Humble - Turtlesim
## Presentado por Juan Esteban Otavo García y Ian Saonni Rodríguez Pulido

En este laboratorio se realizó un primer acercamiento al framework de desarrollo ROS 2 mediante el uso de la aplicación turtlesim. Durante la práctica se llevó a cabo la instalación y configuración inicial del sistema operativo Ubuntu, requerido para el funcionamiento de ROS2. 

Posteriormente, se realizó una pequeña implementación de robótica móvil utilizando turtlesim. En dicha implementación, la tortuga fue controlada desde un nodo independiente mediante un script en Python el cuál utilizó lectura de entradas de teclado para controlar el movimiento de la tortuga y generar diversas trayectorias y movimientos primitivos.

## Procedimiento

Se inició con la instalación y configuración del framework ROS 2 Humble siguiendo las instrucciones descritas en la guía del laboratorio y se habilitó el entorno de trabajo mediante el comando `source /opt/ros/humble/setup.bash`, este comando debe ser utilizado cada vez que se abre una nueva terminal. Luego, se instaló el simulador turtlesim y el software rqt para la monitorización de la implementación.

Con ROS 2 correctamente instalado, se creó un workspace destinado al desarrollo de la práctica. Luego se realizaron las primeras pruebas con turtlesim, un simulador sencillo utilizado para introducir los conceptos fundamentales de ROS. En una primera terminal se ejecutó el nodo principal del simulador y se verificó su comunicación mediante el uso de el nodo turtle_teleop_key, el cuál funcionó como un primer acercamiento al control del movimiento de la tortuga.

Una vez completados los pasos de la guía base, se diseñó e implementó un nodo propio en Python, basado en el nodo move_turtle.py presentado en la guía, destinado a controlar la tortuga mediante entradas de teclado, utilizando las flechas para movimientos primitivos de traslación y rotación.

El nodo implementado interactúa directamente con el tópico `/turtle1/cmd_vel` para publicar las velocidades lineales y angulares requeridas por cada movimiento y lo envía al nodo `/turtle1`, encargado de la simulación. También se utiliza el servicio `/reset` de turtlesim para limpiar la pantalla y reposicionar la tortuga antes de iniciar el dibujo de cada letra.

Este proceso permitió familiarizarse con el entorno y flujo de trabajo de ROS 2, la identificación de nodos y tópicos, así como la interacción entre distintos componentes dentro de un mismo entorno.

## Diagrama de flujo

flowchart TD

    %% NODO PRINCIPAL
    A[Inicio del programa] --> B[Inicializar ROS2<br/>rclpy.init()]
    B --> C[Inicializar curses<br/>initscr(), noecho(), keypad(True)]
    C --> D[Crear nodo<br/>TurtleTeleop]
    D --> E[Crear publisher /turtle1/cmd_vel<br/>y cliente /reset]
    E --> F[Configurar timer<br/>read_keyboard() cada 0.1 s]
    F --> G[[Bucle principal<br/>rclpy.spin(node)]]

    %% TIMER DE LECTURA
    G --> H[Timer dispara<br/>read_keyboard()]
    H --> I[_get_last_key():<br/>leer todas las teclas<br/>y quedarse con la última]

    %% DECISIÓN DE TECLA
    I --> J{¿Tecla válida?}
    J --> K[No hay tecla<br/>key == -1]
    J --> L[Tecla de flecha]
    J --> M[Tecla de letra<br/>I,S,R,P,J,E,O,G]

    %% SIN TECLA
    K --> K1[Construir Twist con<br/>velocidades 0,0]
    K1 --> N[Publicar Twist<br/>/turtle1/cmd_vel]
    N --> G

    %% FLECHAS
    L --> L1[Asignar velocidades:<br/>↑/↓ -> linear.x<br/>←/→ -> angular.z]
    L1 --> N

    %% LETRAS
    M --> N
    M --> O{¿Qué letra es?}
    O --> OI[Llamar draw_I()]
    O --> OS[Llamar draw_S()]
    O --> OR[Llamar draw_R()]
    O --> OP[Llamar draw_P()]
    O --> OJ[Llamar draw_J()]
    O --> OE[Llamar draw_E()]
    O --> OO[Llamar draw_O()]
    O --> OG[Llamar draw_G()]

    %% DIBUJO DE LETRAS (GENÉRICO)
    subgraph DIBUJO_AUTOMÁTICO [Función draw_X()]
        direction TB
        P1[Log: 'Dibujando X'] --> P2[_drain_keys():<br/>vaciar buffer de teclado]
        P2 --> P3[Llamar reset_turtle():<br/>servicio /reset + pausa]
        P3 --> P4[Ejecutar secuencia<br/>de movimientos primitivos]

        subgraph MOV_PRIMITIVOS [Movimientos primitivos]
            direction TB
            Q1[move_forward(t) /<br/>move_backward(t) /<br/>turn_left(t) / turn_right(t)]
            Q1 --> Q2[Construir Twist<br/>con velocidad fija]
            Q2 --> Q3[while tiempo < t]
            Q3 --> Q4[Publicar Twist]
            Q4 --> Q5[_drain_keys():<br/>descartar teclas mientras se mueve]
            Q5 --> Q6[dormir 0.01 s]
            Q6 --> Q3
            Q3 --> Q7[Al terminar:<br/>llamar stop()]
        end
    end

    OI --> P1
    OS --> P1
    OR --> P1
    OP --> P1
    OJ --> P1
    OE --> P1
    OO --> P1
    OG --> P1

    P4 --> G
