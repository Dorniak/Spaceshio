# spaceship_controller — Práctica ROS2

**Grado en Ciencia de Datos · Robótica · 3º curso**

---

## Objetivo

Implementar un controlador autónomo que lleve la nave espacial 2D al punto
objetivo **lo más rápido posible**, cumpliendo las condiciones de llegada:

- Distancia al target **< 2 metros**
- Velocidad de la nave **< 0.1 m/s**

El cronómetro arranca con el primer impulso a cualquier motor y se detiene
al cumplirse ambas condiciones. **Menor tiempo = mejor nota.**

---

## Lo que debéis implementar

Todo vuestro trabajo va en **`spaceship_controller/controller_node.py`**,
concretamente en el método `control_loop()` y las variables de estado
de `__init__()`. El resto del paquete (suscriptores, publicadores, launch)
ya está preparado.

---

## Instalación

```bash
# Copiar ambos paquetes al workspace (el profesor os dará spaceship_sim)
cp -r spaceship_sim      ~/ros2_ws/src/
cp -r spaceship_controller ~/ros2_ws/src/

cd ~/ros2_ws
colcon build --packages-select spaceship_sim spaceship_controller
source install/setup.bash
```

---

## Uso

```bash
# Sin target inicial (declarar con click en RViz):
ros2 launch spaceship_controller full_sim.launch.py

# Con target inicial (el controlador arranca automáticamente):
ros2 launch spaceship_controller full_sim.launch.py target_x:=10.0 target_y:=8.0
```

Para cambiar el target en cualquier momento: selecciona la herramienta
**Publish Point** en la barra superior de RViz y haz click en el mapa.

---

## Estructura del paquete

```
spaceship_controller/
├── spaceship_controller/
│   └── controller_node.py   ← AQUÍ va todo vuestro código
├── launch/
│   ├── full_sim.launch.py   ← simulador + controlador
│   └── controller.launch.py ← solo el controlador
├── package.xml
└── setup.py
```

---

## Topics

| Topic | Dirección | Tipo | Descripción |
|-------|-----------|------|-------------|
| `/ship_state` | → suscribir | `ShipState` | Estado de la nave a 20 Hz |
| `/clicked_point` | → suscribir | `PointStamped` | Target desde RViz |
| `/motor_command` | ← publicar | `MotorCommand` | Comandos a los motores |

### ShipState — campos disponibles

```
float64 x, y          → posición (metros)
float64 heading        → orientación (rad): 0=este, π/2=norte
float64 vx, vy         → velocidad (m/s)
float64 omega          → velocidad angular (rad/s)
float32 power_m1       → potencia motor izquierdo actual (0-100)
float32 power_m2       → potencia motor derecho actual (0-100)
float64 elapsed_time   → tiempo desde primer impulso (s)
bool    arrived        → True al cumplirse condiciones de llegada
float64 target_x/y     → coordenadas del target actual
```

### MotorCommand

```
uint8 motor_id   → 1=izquierdo, 2=derecho, 0=ambos igual
uint8 power      → 0-100 (%)
```

---

## Modelo físico — T invertida

```
        ↑ heading (nariz)
        |
   [M1]———[M2]

M1 solo  (motor_id=1)  →  gira a la DERECHA
M2 solo  (motor_id=2)  →  gira a la IZQUIERDA
M1 = M2  (motor_id=0)  →  avance RECTO
M1 ≠ M2               →  avance + giro (diferencial)
```

La nave tiene **inercia**: si apagáis los motores, sigue moviéndose.
Tendréis que anticipar el frenado.

---

## Pistas y sugerencias

### 1. Geometría útil

```python
import math

dx = self.target_x - self.state.x
dy = self.target_y - self.state.y

dist            = math.sqrt(dx**2 + dy**2)        # distancia al target
angle_to_target = math.atan2(dy, dx)              # ángulo al target (rad)
heading_error   = angle_diff(angle_to_target, self.state.heading)
# heading_error > 0 → target a la izquierda → activar M2
# heading_error < 0 → target a la derecha   → activar M1
```

### 2. Estrategia mínima funcional (Bang-Bang)

Una estrategia sencilla que funciona:

```
ORIENT → girar hasta apuntar al target
THRUST → impulsar a máxima potencia
BRAKE  → frenar antes de llegar
FINE   → ajuste final a baja potencia
```

### 3. ¿Cómo frenar?

La nave no tiene frenos. Para decelerarla hay que:
1. Orientarla **180°** respecto al target (apuntar hacia atrás)
2. Encender los motores (el empuje va ahora en sentido contrario al movimiento)

Clave: anticipar **cuándo** girar. Si esperáis demasiado, pasaréis el target.

### 4. Corrección de heading durante el avance

Si durante la fase de impulso la nave se desvía ligeramente, podéis
corregir desbalanceando los motores:

```python
# Corrección proporcional al error de heading
correccion = heading_error * KP * POWER_MAX
m1 = POWER_MAX - max(0,  correccion)   # reducir M1 si hay que girar izq
m2 = POWER_MAX + min(0, correccion)   # reducir M2 si hay que girar der
```

### 5. Ampliaciones (para nota extra)

- **Frenado predictivo**: calcular `d_stop = v² / (2·a_max)` y activar
  el frenado exactamente cuando `dist ≈ d_stop`
- **Controlador PID**: sustituir el bang-bang por un PID completo
  con `Kp`, `Ki`, `Kd` e integrador anti-windup
- **Trayectoria óptima**: ¿merece la pena girar primero o moverse en diagonal?

---

## Autores

Grupo X — [Nombre1, Nombre2, Nombre3, Nombre4]
