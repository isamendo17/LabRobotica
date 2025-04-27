# LabRobotica 2025 - 1
El siguiente Github muestra la resolución de los laboratorios planteados en la asignatura de Robótica de la Universidad Nacional de Colombia.

Integrantes:
- Andrés Santiago Cañon Porras
- Isabella Mendoza Cáceres

## Laboratorio No. 1

```mermaid
flowchart TD
    A[Inicio] --> B[Imprimir controles en pantalla]
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

