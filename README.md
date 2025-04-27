# LabRobotica 2025 - 1
El siguiente Github muestra la resolución de los laboratorios planteados en la asignatura de Robótica de la Universidad Nacional de Colombia.

Integrantes:
- Andrés Santiago Cañon Porras
- Isabella Mendoza Cáceres

## Laboratorio No. 1
**Turtle Controller en ROS 2**

### Objetivos
El objetivo de este laboratorio es controlar una tortuga en el simulador `turtlesim` utilizando ROS 2. En el código se implementa un nodo llamado `turtle_controller`, que permite mover la tortuga de forma manual con el teclado y dibujar las letras de las iniciales de los estudiantes (A, S, C, P, I y M) en la pantalla. Además, se integra una función para limpiar la pantalla del simulador.

*Librerías utilizadas*
- `rclpy`: Permite programar nodos en Python utilizando ROS 2.
- `rclpy.node.Node`: Clase base para crear nodos ROS 2.
- `std_srvs.srv.Empty`: Servicio estándar utilizado para limpiar la pantalla del `turtlesim`.
- `geometry_msgs.msg.Twist`: Mensaje utilizado para enviar comandos de movimiento lineal y angular a la tortuga.
- `turtlesim.msg.Pose`: Mensaje que proporciona la posición y orientación actual de la tortuga.
- `turtlesim.srv.TeleportRelative`, `turtlesim.srv.TeleportAbsolute`, `turtlesim.srv.SetPen`: Servicios de `turtlesim` que permiten teletransportar la tortuga y controlar el estado del lápiz (encendido/apagado).
- `threading`: Librería para manejar hilos de ejecución concurrentes.
- `curses`: Librería para gestionar la entrada del teclado de forma no bloqueante en consola.
- `time`, `math`: Librerías estándar para temporizaciones y cálculos matemáticos.

### Descripción de las funciones principales
- **`__init__`**: Inicializa el nodo, crea el publicador para enviar comandos de movimiento y suscripciones a la posición de la tortuga.
- **`pose_callback`**: Actualiza las variables internas `x` y `y` con la posición actual de la tortuga.
- **`clear_screen`**: Llama al servicio `/clear` para limpiar la pantalla del simulador.
- **`set_abs_angle`**: Teletransporta a la tortuga a su posición actual pero cambiando su ángulo absoluto.
- **`set_pen`**: Enciende o apaga el lápiz que deja la estela de la tortuga.
- **`move_turtle` y `move_turtle2`**: Controlan el movimiento de la tortuga basado en la acción actual (teclas presionadas o dibujo).
- **`move_forward`**: Mueve la tortuga hacia adelante a una velocidad y tiempo determinados.
- **`drawCharacter`**: Implementa la lógica para dibujar diferentes letras (`M`, `A`, `S`, `I`, `C`, `P`) mediante combinaciones de movimientos y rotaciones.
- **`rotate_turtle`**: Rota a la tortuga un ángulo relativo especificado usando el servicio de teletransporte relativo.
- **`read_keys`**: Usa la librería `curses` para leer entradas de teclado de manera continua y no bloqueante. Asocia teclas específicas a acciones de movimiento o de dibujo.
- **`run_curses` y `main`**: Lanzan la lectura de teclado en un hilo separado y ejecutan el nodo de ROS 2 de manera continua.

### Procedimiento realizado
1. Se crea un nodo de ROS 2 llamado `turtle_controller`.
2. Se establece un publicador para controlar la velocidad de la tortuga y un suscriptor para conocer su posición actual.
3. Se implementan servicios para limpiar la pantalla, rotar la tortuga y controlar el lápiz.
4. Se desarrolla una función que permite mover la tortuga en distintas direcciones mediante el teclado y dibujar letras específicas mediante combinaciones de movimientos programados.
5. Se utiliza `curses` para leer las teclas presionadas sin bloquear la ejecución del programa, manteniendo la interactividad.
6. Se manejan múltiples hilos para poder controlar simultáneamente el movimiento, dibujo y lectura de entradas.

### Decisiones de diseño
- **Uso de hilos (`threading`)**: Para poder controlar los comandos de teclado y el movimiento de la tortuga sin bloquear la actualización de ROS 2.
- **Separación de funciones**: Cada acción específica (mover, rotar, dibujar) se realiza en funciones separadas para mantener un diseño modular y fácil de mantener.
- **Uso de `curses`**: Se eligió esta librería para capturar eventos de teclado de forma no bloqueante en consola, permitiendo una respuesta más fluida al usuario.
- **Servicios de teletransporte**: Para posicionar y rotar a la tortuga de manera precisa, se emplean los servicios `TeleportRelative` y `TeleportAbsolute` en lugar de comandos de velocidad tradicionales.

### Funcionamiento general
Entonces, como funcionamiento general se tiene que el usuario ejecuta el nodo `turtle_controller`. Desde la terminal, mediante las teclas de dirección, puede mover a la tortuga hacia adelante, atrás o girarla. También puede presionar letras específicas (`M`, `A`, `S`, `I`, `C`, `P`) para que la tortuga dibuje esa letra en la pantalla del `turtlesim`. Adicionalmente, la tecla `V` limpia la pantalla. Todo esto ocurre mientras el nodo mantiene actualizada la posición y estado de la tortuga en tiempo real.


### Diagrama de flujo

```mermaid
flowchart TD
    A[Inicio] --> B[Imprimir información y controles en pantalla]
    B --> C[Leer tecla]
    C --> D{¿Tecla presionada?}
    D -- No --> C
    D -- Sí --> E{¿Tecla de dirección?}
    E -- Sí --> F{¿Flecha arriba?}
    E -- No --> G{¿Letra A, S, C, P, I o M?}
    F -- Sí --> H[Mover hacia adelante]
    F -- No --> I{¿Flecha abajo?}
    G -- Sí --> J[Dibujar letra]
    G -- No --> K{¿Letra V?}
    H --> C
    I -- Sí --> L[Mover hacia abajo]
    I -- No --> M{¿Flecha izquierda?}
    J --> C
    K -- Sí --> N[Limpiar pantalla]
    K -- No --> C
    L --> C
    M -- Sí --> O[Girar a la izquierda]
    M -- No --> P{¿Flecha derecha?}
    N --> C
    O --> C
    P -- Sí --> Q[Girar a la derecha]
    P -- No --> C
    Q --> C
```

