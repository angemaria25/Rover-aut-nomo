# Proyecto 1 — Rover autónomo (Elementos de Inteligencia Artificial)

Simulación de un rover que explora un entorno discreto con recursos limitados
(batería y memoria) y resuelve los problemas de la misión con técnicas de IA
clásica. El proyecto implementa tres módulos obligatorios/elegidos más un
módulo opcional bonificable:

| Módulo | Archivo | Descripción |
|---|---|---|
| A. Búsqueda | `src/rover_ai/search.py` | BFS, DFS, Dijkstra, A* y voraz; rutas entre objetivos con costo variable por terreno. |
| B. Planificación | `src/rover_ai/planning.py` | Acciones con precondiciones y efectos; integra la búsqueda para cada movimiento. |
| D. Metaheurísticas | `src/rover_ai/metaheuristics.py` | Hill Climbing y Recocido Simulado para el orden de visita de los POI. |
| E. Reglas | `src/rover_ai/rules.py` | Motor de encadenamiento hacia adelante para las decisiones del rover. |

Apoyo: `environment.py` (generador de instancias por semilla, parametrizable en
tamaño, estructura y puntos de carga, y con solución garantizada), `simulation.py`
(ejecución/validación de planes y puntuación de misión), `experiments.py`
(protocolo experimental) y `cli.py` (entrada por consola).

## Estructura

```
proyecto/
├── data/                 # instancias de ejemplo
├── docs/                 # informe (Markdown, HTML y PDF) y figuras
├── notebooks/            # análisis de resultados
├── outputs/              # resultados generados (CSV y JSON)
├── src/rover_ai/         # código fuente del paquete
├── pyproject.toml        # entorno declarado (empaquetado)
├── requirements.txt      
└── README.md
```

## Requisitos

- Python 3.11 o superior (probado con 3.11+).
- Solo la librería estándar de Python.

## Cómo ejecutar desde cero

1. Crear y activar un entorno virtual:

   ```powershell
   python -m venv .venv
   .venv\Scripts\activate            # Windows
   # source .venv/bin/activate       # Linux/macOS
   ```

2. Instalar el paquete en modo editable (deja disponible `rover_ai`):

   ```powershell
   python -m pip install -e .
   ```

   > Alternativa sin instalar: anteponer `src` al `PYTHONPATH`.
   > PowerShell: `$env:PYTHONPATH="src"; python -m rover_ai.cli run-all`
   > Bash:       `PYTHONPATH=src python -m rover_ai.cli run-all`

3. Ejecutar todo el protocolo experimental:

   ```powershell
   python -m rover_ai.cli run-all
   ```

   Esto escribe en `outputs/`: `search_results.csv`, `structure_results.csv`,
   `metaheuristic_results.csv`, `planning_results.csv`, `rules_results.csv`,
   `mission_demo.json` y `summary.json`.

## Comandos disponibles

```powershell
python -m rover_ai.cli run-all               # protocolo experimental completo
python -m rover_ai.cli benchmark-search      # solo búsqueda (5 algoritmos)
python -m rover_ai.cli benchmark-structures  # búsqueda en 3 estructuras de terreno
python -m rover_ai.cli benchmark-meta        # solo metaheurísticas
python -m rover_ai.cli benchmark-planning    # óptimo vs orden de metaheurística
python -m rover_ai.cli benchmark-rules       # módulo E: política basada en reglas
python -m rover_ai.cli mission-demo --seed 23
python -m rover_ai.cli plan-demo --seed 23   # muestra un plan y su validación
python -m rover_ai.cli rules-demo --seed 23  # módulo E: política basada en reglas
python -m rover_ai.cli summarize             # agrega los CSV en summary.json
python -m rover_ai.cli save-instance --seed 23 --out data/instancia_ejemplo.json
```

## Notebook (opcional)

El notebook `notebooks/analisis_resultados.ipynb` solo lee los resultados de
`outputs/` y muestra las tablas. Para ejecutarlo en VS Code hace falta el kernel
de Jupyter en el entorno virtual:

```powershell
python -m pip install -e ".[notebook]"
```

Luego abre el notebook y selecciona el kernel del `.venv`.

## Modelado del problema

- **Entorno**: rejilla con obstáculos y cinco terrenos transitables con costo
  creciente (llano 1, piedra 2, roca 3, arena fina 4, duna 5). El costo se paga
  al entrar a la celda.
- **Generador parametrizable**: tamaño (`width`, `height`), estructura
  (`terrain_profile`: `flat` / `default` / `rough`, y `obstacle_prob`) y puntos
  de carga (`n_charging_stations`).
- **Tableros con solución**: el generador garantiza que todos los puntos clave
  sean alcanzables y que estén dentro del alcance de la batería (abre corredores
  en L si hace falta), de modo que ninguna instancia sea irresoluble.
- **Recursos**: batería (capacidad 100) y memoria (3 muestras sin transmitir).
  Se recarga en la base o en cualquier **estación de carga**.
- **Objetivos**: POI (muestras) que hay que recolectar, analizar y transmitir
  desde una zona de comunicación.
- **Puntuación de misión**: `recompensa · muestras_transmitidas − costo_total`.

## Protocolo experimental

- Instancias generadas por semilla con el generador del entorno (nunca un mapa
  fijo). Distintos tamaños por módulo.
- Semillas declaradas en `experiments.py` (`SEARCH_SEEDS`, `META_SEEDS`,
  `PLANNING_SEEDS`, `RULES_SEEDS`; `MISSION_SEED` fija para la demo).
- Algoritmos con componente aleatoria: varias corridas por instancia con
  semillas distintas.
- **Mismo presupuesto de cómputo** para Hill Climbing y Recocido Simulado
  (mismo número de evaluaciones de la función objetivo).
- Se reporta **media y desviación**, no la mejor corrida.

## Reproducibilidad

- El generador de instancias es determinista dada la semilla.
- Todas las semillas están fijadas en el código (`experiments.py`).
- `python -m rover_ai.cli run-all` regenera todos los resultados desde cero.

## Informe

En `docs/` está el informe del proyecto:

- `docs/figuras/` — figuras y el script que las regenera (`generar_figuras.py`).

Para reconstruir el informe:

```powershell
python docs/figuras/generar_figuras.py   # regenera las figuras (lee outputs/summary.json)
python docs/build_informe.py             # informe.md -> informe.html
# y luego imprimir informe.html a PDF (p. ej. con el navegador)
```

