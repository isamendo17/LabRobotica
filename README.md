# LabRobotica 2025 - 1
El siguiente Github muestra la resolución de los laboratorios planteados en la asignatura de Robótica de la Universidad Nacional de Colombia.

Integrantes:
- Andrés Santiago Cañon Porras
- Isabella Mendoza Cáceres

## Laboratorio No. 1

### Diagrama de flujo

```mermaid
flowchart TD
    A[Inicio] --> B[Imprimir información y controles en pantalla]
    B --> C[Leer tecla]
    C --> D{¿Tecla presionada?}
    D -- No --> C
    D -- Sí --> E{¿Tecla de dirección?}

```

