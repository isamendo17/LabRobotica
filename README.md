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
    C --> D{Tecla presionada?}
    D -- Sí --> E{Tecla de dirección?}
    D -- No --> C
    E -- Sí --> F{Flecha arriba?}
    E -- No --> G{Letras A, S, C, P, I o M?}
    F -- Sí --> H[Mover hacia adelante]
    F -- No --> I{Flecha abajo?}
    G -- Sí --> J[Dibujar letra]
    G -- No --> K{Letra V?}
    I -- Sí --> L[Mover hacia abajo]
    I -- No --> M{Flecha izquierda?}
    K -- Sí --> N[Limpiar pantalla]
    K -- No --> C
    M -- Sí --> O[Girar a la izquierda]
    M -- No --> P{Flecha derecha?}
    P -- Sí --> Q[Girar a la derecha]
    P -- No --> C
```
