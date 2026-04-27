#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════╗
║   SPACESHIP CONTROLLER — Implementación de referencia (Profesor)    ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                      ║
║   Tres estrategias seleccionables desde el launch:                  ║
║     · bangbang  — Máquina de estados con corrección proporcional     ║
║     · pid       — PID de heading + frenado predictivo                ║
║     · fuzzy     — Lógica difusa (orientación + propulsión + frenado) ║
║                                                                      ║
║   Parámetro de selección:                                            ║
║     strategy:=bangbang | pid | fuzzy                                 ║
║                                                                      ║
║   Convención de motores:                                             ║
║     M1 = motor IZQUIERDO  (Y-)  torque = (F1-F2)*ARM → si M1>M2 gira derecha ║
║     M2 = motor DERECHO    (Y+)  torque = (F1-F2)*ARM → si M2>M1 gira izquierda ║
║                                                                      ║
║   NOTA FÍSICA DEL SIMULADOR:                                         ║
║     ft = (f1+f2)/2  →  aceleración máxima ≈ MAX_THRUST/2 = 1.5 m/s²║
║     torque = (F1-F2)*ARM_LENGTH  (F1=M1, F2=M2)                     ║
║     herr>0 → target a IZQUIERDA → necesita girar izq → M2 sube      ║
║       porque torque>0 (F2>F1) gira izquierda                        ║
║                                                                      ║
╚══════════════════════════════════════════════════════════════════════╝
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from spaceship_msgs.msg import MotorCommand, ShipState

_NO_TARGET = -999.0

# ── Física del simulador (deben coincidir con ship_simulator.py) ──────
MAX_THRUST   = 3.0   # N/kg por motor
ARM_LENGTH   = 0.7
LINEAR_DRAG  = 0.5
ANGULAR_DRAG = 1.2
INERTIA      = 1.5

# Aceleración máxima de traslación (ambos motores al 100%)
# ft = (f1+f2)/2 = MAX_THRUST  →  a_max = MAX_THRUST - v*LINEAR_DRAG
# A velocidad baja: a_max ≈ MAX_THRUST = 3.0 m/s²
A_MAX = MAX_THRUST  # 3.0 m/s²


# ══════════════════════════════════════════════════════════════════════
#  UTILIDADES COMUNES
# ══════════════════════════════════════════════════════════════════════

def angle_diff(a: float, b: float) -> float:
    """Diferencia angular normalizada a [-π, π]."""
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def braking_distance(speed: float, margin: float = 1.2) -> float:
    """
    Distancia de frenado estimada con drag lineal.
    Integración analítica aproximada de v²/(2*(A_MAX - drag*v)).
    Para velocidades bajas el drag es pequeño → usa A_MAX.
    Para velocidades altas el drag ayuda → distancia real menor.
    Usamos conservadoramente A_MAX/2 (frenado efectivo real es menor
    porque el drag frena también, pero así tenemos margen).
    """
    a_eff = max(A_MAX * 0.5, A_MAX - LINEAR_DRAG * speed)
    return (speed * speed) / (2.0 * a_eff) * margin


# ══════════════════════════════════════════════════════════════════════
#  ESTRATEGIA 1: BANG-BANG CON CORRECCIÓN PROPORCIONAL
# ══════════════════════════════════════════════════════════════════════

class BangBangController:
    """
    Máquina de estados:
        IDLE    → sin target
        ORIENT  → girar hasta apuntar al target
        THRUST  → impulsar corrigiendo heading en vuelo
        BRAKE   → orientar 180° y frenar
        FINE    → ajuste suave final

    Convención de giro (según simulador):
        torque = (F1 - F2) * ARM_LENGTH
        F1 > F2 → torque > 0 → gira DERECHA (heading decrece)
        F2 > F1 → torque < 0 → gira IZQUIERDA (heading aumenta)

        herr > 0 → target a la IZQUIERDA → queremos girar izq → M2 sube (F2 > F1)
        herr < 0 → target a la DERECHA   → queremos girar der → M1 sube (F1 > F2)
    """

    ANGLE_TOL      = 0.10   # rad — tolerancia de orientación (~6°)
    BRAKE_TOL      = 0.20   # rad — tolerancia alineación en frenado (más laxa)
    POWER_MAX      = 80     # % — potencia máxima de empuje
    POWER_BRAKE    = 80     # % — potencia de frenado
    POWER_FINE     = 25     # % — potencia en ajuste fino
    POWER_TURN     = 60     # % — potencia de giro puro
    KP_HEADING     = 40.0   # ganancia proporcional heading en vuelo
    BRAKE_MARGIN   = 1.3    # factor de seguridad distancia frenado
    FINE_DISTANCE  = 2.0    # m — umbral zona fina (= ARRIVAL_DIST del sim)
    FINE_SPEED     = 0.5    # m/s — velocidad máxima aceptable en zona fina

    def __init__(self):
        self.phase = 'IDLE'

    def reset(self):
        self.phase = 'ORIENT'

    def compute(self, s: ShipState, tx: float, ty: float) -> tuple[int, int]:
        dx    = tx - s.x
        dy    = ty - s.y
        dist  = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx)
        herr  = angle_diff(angle, s.heading)
        speed = math.sqrt(s.vx**2 + s.vy**2)

        # Velocidad radial hacia el target (positiva = acercándose)
        vr = (s.vx * dx + s.vy * dy) / max(dist, 0.1)

        # Distancia de frenado necesaria (solo si nos acercamos)
        d_stop = braking_distance(max(vr, 0.0), self.BRAKE_MARGIN)

        if self.phase == 'ORIENT':
            if abs(herr) < self.ANGLE_TOL:
                self.phase = 'THRUST'
                return self.POWER_MAX, self.POWER_MAX
            return self._turn(herr)

        elif self.phase == 'THRUST':
            # Entrar en frenado si la distancia de parada supera la distancia al target
            if dist < self.FINE_DISTANCE:
                self.phase = 'FINE'
                return 0, 0
            if vr > 0.1 and d_stop >= dist:
                self.phase = 'BRAKE'
                return 0, 0
            # Corrección proporcional de heading durante empuje
            corr = clamp(herr * self.KP_HEADING, -self.POWER_MAX, self.POWER_MAX)
            # herr>0 → izquierda → M2 sube
            m1 = clamp(self.POWER_MAX - corr, 0, self.POWER_MAX)
            m2 = clamp(self.POWER_MAX + corr, 0, self.POWER_MAX)
            return int(m1), int(m2)

        elif self.phase == 'BRAKE':
            if dist < self.FINE_DISTANCE:
                self.phase = 'FINE'
                return 0, 0
            # Ya no nos acercamos o velocidad baja → pasar a FINE
            if vr < 0.2 and speed < self.FINE_SPEED:
                self.phase = 'FINE'
                return 0, 0
            # Orientar 180° (apuntar hacia atrás respecto al target)
            brake_angle = math.atan2(-dy, -dx)
            berr = angle_diff(brake_angle, s.heading)
            if abs(berr) < self.BRAKE_TOL:
                # Alineado: frenar simétricamente
                return self.POWER_BRAKE, self.POWER_BRAKE
            return self._turn(berr)

        elif self.phase == 'FINE':
            # Condición de llegada
            if speed < 0.08 and dist < self.FINE_DISTANCE:
                return 0, 0

            if speed > self.FINE_SPEED:
                # Frenar activamente: apuntar contra la velocidad
                vel_angle = math.atan2(s.vy, s.vx)
                brake_angle = vel_angle + math.pi  # opuesto a la velocidad
                berr = angle_diff(brake_angle, s.heading)
                if abs(berr) < self.BRAKE_TOL * 1.5:
                    return self.POWER_FINE, self.POWER_FINE
                return self._turn_fine(berr)

            # Velocidad baja: reorientar y empujar suave hacia target
            if dist > 0.5 and abs(herr) > self.ANGLE_TOL * 2:
                return self._turn_fine(herr)
            if dist > 0.5:
                corr = clamp(herr * self.KP_HEADING * 0.3, -self.POWER_FINE, self.POWER_FINE)
                m1 = clamp(self.POWER_FINE - corr, 0, self.POWER_FINE)
                m2 = clamp(self.POWER_FINE + corr, 0, self.POWER_FINE)
                return int(m1), int(m2)
            return 0, 0

        return 0, 0   # IDLE o fallback

    def _turn(self, herr: float) -> tuple[int, int]:
        """Giro puro. herr>0 → izquierda → M2 sube."""
        if herr > 0:
            return 0, self.POWER_TURN
        else:
            return self.POWER_TURN, 0

    def _turn_fine(self, herr: float) -> tuple[int, int]:
        """Giro suave en zona fina."""
        p = int(self.POWER_TURN * 0.5)
        if herr > 0:
            return 0, p
        else:
            return p, 0


# ══════════════════════════════════════════════════════════════════════
#  ESTRATEGIA 2: PID
# ══════════════════════════════════════════════════════════════════════

class PIDController:
    """
    Controlador PID dual:
        · PID de heading   → diferencial de motores (giro)
        · Velocidad base   → potencia proporcional a distancia

    Frenado predictivo basado en distancia de parada real.

    Convención:
        herr > 0 → target izquierda → M2 sube (output positivo → M2 sube)
    """

    # ── Parámetros PID heading ────────────────────────────────────────
    KP_H    = 60.0
    KI_H    = 1.5
    KD_H    = 10.0
    I_LIM_H = 30.0

    # ── Parámetros de maniobra ────────────────────────────────────────
    BRAKE_MARGIN  = 1.25   # factor de seguridad distancia frenado
    FINE_DIST     = 2.0    # m — zona de aproximación final
    POWER_BASE    = 70     # % — potencia de crucero
    POWER_BRAKE   = 85     # % — potencia de frenado
    POWER_FINE    = 22     # % — potencia en zona fina
    ANGLE_TOL     = 0.10   # rad
    BRAKE_TOL     = 0.20   # rad

    def __init__(self):
        self._reset_pid()
        self.phase = 'IDLE'

    def _reset_pid(self):
        self.int_h  = 0.0
        self.prev_h = 0.0

    def reset(self):
        self.phase = 'ORIENT'
        self._reset_pid()

    def compute(self, s: ShipState, tx: float, ty: float,
                dt: float = 0.05) -> tuple[int, int]:
        dx    = tx - s.x
        dy    = ty - s.y
        dist  = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx)
        herr  = angle_diff(angle, s.heading)
        speed = math.sqrt(s.vx**2 + s.vy**2)
        vr    = (s.vx * dx + s.vy * dy) / max(dist, 0.1)

        d_stop = braking_distance(max(vr, 0.0), self.BRAKE_MARGIN)

        if self.phase == 'ORIENT':
            if abs(herr) < self.ANGLE_TOL:
                self.phase = 'THRUST'
                self._reset_pid()
            return self._pid_turn(herr, dt)

        elif self.phase == 'THRUST':
            if dist < self.FINE_DIST:
                self.phase = 'FINE'
                self._reset_pid()
                return 0, 0
            if vr > 0.1 and d_stop >= dist:
                self.phase = 'BRAKE'
                self._reset_pid()
                return 0, 0
            return self._pid_thrust(herr, dt, self.POWER_BASE)

        elif self.phase == 'BRAKE':
            if dist < self.FINE_DIST:
                self.phase = 'FINE'
                self._reset_pid()
                return 0, 0
            if vr < 0.2 and speed < 0.5:
                self.phase = 'FINE'
                self._reset_pid()
                return 0, 0
            # Orientar contra la velocidad actual (más fiable que -target)
            vel_angle   = math.atan2(s.vy, s.vx)
            brake_angle = vel_angle + math.pi
            berr = angle_diff(brake_angle, s.heading)
            if abs(berr) < self.BRAKE_TOL:
                # Frenado simétrico: potencia fija sin corrección diferencial
                p = self.POWER_BRAKE
                return p, p
            return self._pid_turn(berr, dt)

        elif self.phase == 'FINE':
            if speed < 0.08 and dist < self.FINE_DIST:
                return 0, 0
            if speed > 0.4:
                vel_angle   = math.atan2(s.vy, s.vx)
                brake_angle = vel_angle + math.pi
                berr = angle_diff(brake_angle, s.heading)
                if abs(berr) < self.BRAKE_TOL * 1.5:
                    return self.POWER_FINE, self.POWER_FINE
                return self._pid_turn(berr, dt)
            return self._pid_thrust(herr, dt, self.POWER_FINE)

        return 0, 0

    def _pid_turn(self, herr: float, dt: float) -> tuple[int, int]:
        """
        Giro puro diferencial (base = 0).
        output > 0 → herr > 0 → target izquierda → M2 sube
        """
        self.int_h = clamp(self.int_h + herr * dt, -self.I_LIM_H, self.I_LIM_H)
        deriv = (herr - self.prev_h) / max(dt, 1e-6)
        self.prev_h = herr
        output = clamp(
            self.KP_H * herr + self.KI_H * self.int_h + self.KD_H * deriv,
            -100, 100
        )
        # output > 0 → izquierda → M2 sube
        m1 = clamp(-output, 0, 100)
        m2 = clamp( output, 0, 100)
        return int(m1), int(m2)

    def _pid_thrust(self, herr: float, dt: float,
                    base_power: int) -> tuple[int, int]:
        """
        Avance con corrección PID de heading.
        corr > 0 → herr > 0 → izquierda → M2 sube
        """
        self.int_h = clamp(self.int_h + herr * dt, -self.I_LIM_H, self.I_LIM_H)
        deriv = (herr - self.prev_h) / max(dt, 1e-6)
        self.prev_h = herr
        corr = clamp(
            self.KP_H * herr + self.KI_H * self.int_h + self.KD_H * deriv,
            -base_power, base_power
        )
        m1 = clamp(base_power - corr, 0, 100)
        m2 = clamp(base_power + corr, 0, 100)
        return int(m1), int(m2)


# ══════════════════════════════════════════════════════════════════════
#  ESTRATEGIA 3: FUZZY LOGIC
# ══════════════════════════════════════════════════════════════════════

class FuzzyController:
    """
    Controlador de lógica difusa Mamdani simplificado.

    Variables de entrada:
        · heading_error  (rad):  NEGBIG, NEGSMALL, ZERO, POSSMALL, POSBIG
        · distance       (m):    CLOSE, MEDIUM, FAR
        · speed          (m/s):  SLOW, MEDIUM_S, FAST

    Salidas:
        · differential   (-100..100): positivo → gira izquierda (M2 sube)
        · thrust         (0..100):    potencia base

    Convención diferencial (según simulador):
        torque = (F1-F2)*ARM  → F2>F1 gira izquierda
        differential > 0 → M2 sube, M1 baja (gira izquierda)
        differential < 0 → M1 sube, M2 baja (gira derecha)
        herr > 0 → target izquierda → differential positivo
    """

    BRAKE_MARGIN = 1.3

    def __init__(self):
        self.phase = 'IDLE'

    def reset(self):
        self.phase = 'RUN'

    @staticmethod
    def _tri(x, a, b, c) -> float:
        if x <= a or x >= c: return 0.0
        if x <= b: return (x - a) / (b - a) if b != a else 1.0
        return (c - x) / (c - b) if c != b else 1.0

    @staticmethod
    def _trap(x, a, b, c, d) -> float:
        if x <= a or x >= d: return 0.0
        if x <= b: return (x - a) / (b - a) if b != a else 1.0
        if x <= c: return 1.0
        return (d - x) / (d - c) if d != c else 1.0

    def _fuzzify_heading(self, herr: float) -> dict:
        h = herr
        return {
            'NEGBIG':   self._trap(h, -math.pi, -math.pi, -0.6, -0.2),
            'NEGSMALL': self._tri(h, -0.5, -0.15, 0.0),
            'ZERO':     self._tri(h, -0.2,  0.0,  0.2),
            'POSSMALL': self._tri(h,  0.0,  0.15, 0.5),
            'POSBIG':   self._trap(h,  0.2,  0.6, math.pi, math.pi),
        }

    def _fuzzify_dist(self, dist: float) -> dict:
        return {
            'CLOSE':  self._trap(dist, 0,    0,    2.0,  4.5),
            'MEDIUM': self._tri(dist,  3.0,  6.0, 10.0),
            'FAR':    self._trap(dist, 7.0, 12.0, 999,  999),
        }

    def _fuzzify_speed(self, speed: float) -> dict:
        return {
            'SLOW':     self._trap(speed, 0,   0,   0.5, 1.5),
            'MEDIUM_S': self._tri(speed,  1.0, 2.5, 4.5),
            'FAST':     self._trap(speed, 3.5, 6.0, 999, 999),
        }

    def compute(self, s: ShipState, tx: float, ty: float,
                dt: float = 0.05) -> tuple[int, int]:
        dx    = tx - s.x
        dy    = ty - s.y
        dist  = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx)
        herr  = angle_diff(angle, s.heading)
        speed = math.sqrt(s.vx**2 + s.vy**2)
        vr    = (s.vx * dx + s.vy * dy) / max(dist, 0.1)

        fh = self._fuzzify_heading(herr)
        fd = self._fuzzify_dist(dist)
        fs = self._fuzzify_speed(speed)

        alineado    = fh['ZERO'] + 0.5 * (fh['POSSMALL'] + fh['NEGSMALL'])
        no_alineado = fh['POSBIG'] + fh['NEGBIG']

        # differential > 0 → gira izquierda → M2 sube
        # herr > 0 → target izquierda → differential positivo
        diff_rules: list[tuple[float, float]] = [
            (fh['NEGBIG'],   -85.0),   # target derecha fuerte → M1 sube
            (fh['NEGSMALL'], -30.0),   # target derecha suave
            (fh['ZERO'],       0.0),   # recto
            (fh['POSSMALL'],  30.0),   # target izquierda suave → M2 sube
            (fh['POSBIG'],    85.0),   # target izquierda fuerte
        ]

        # Distancia de frenado necesaria
        d_stop = braking_distance(max(vr, 0.0), self.BRAKE_MARGIN)
        # need_brake: 1 cuando d_stop ≥ dist y nos acercamos
        if vr > 0.2 and dist > 0:
            need_brake = clamp((d_stop - dist) / max(dist * 0.5, 1.0), 0, 1)
        else:
            need_brake = 0.0

        thrust_rules: list[tuple[float, float]] = [
            (min(fd['FAR'],    fs['SLOW'],     alineado),    75.0),
            (min(fd['FAR'],    fs['MEDIUM_S'], alineado),    55.0),
            (min(fd['FAR'],    fs['FAST'],     alineado),    35.0),
            (min(fd['MEDIUM'], fs['SLOW'],     alineado),    50.0),
            (min(fd['MEDIUM'], fs['MEDIUM_S'], alineado),    25.0),
            (min(fd['MEDIUM'], fs['FAST'],                1), 8.0),
            (min(fd['CLOSE'],  fs['FAST'],                1),  0.0),
            (min(fd['CLOSE'],  fs['SLOW'],     alineado),      15.0),
            (min(no_alineado,                             1),   5.0),
            (need_brake,                                         0.0),
        ]

        def wm(rules):
            num = sum(st * v for st, v in rules)
            den = sum(st     for st, _ in rules)
            return num / den if den > 1e-9 else 0.0

        differential = wm(diff_rules)
        thrust       = wm(thrust_rules)

        # Frenado activo: cuando need_brake es alto, ignorar reglas normales
        # y orientar contra la velocidad actual
        if need_brake > 0.5 and speed > 0.4:
            vel_angle   = math.atan2(s.vy, s.vx)
            brake_angle = vel_angle + math.pi  # opuesto a la velocidad
            berr = angle_diff(brake_angle, s.heading)
            # Diferencial para girar hacia brake_angle
            # berr > 0 → brake_angle está a izquierda → M2 sube → differential positivo
            differential = clamp(berr * 80.0, -100, 100)
            if abs(berr) < 0.25:
                thrust = 70.0  # frenar fuerte
            else:
                thrust = 0.0   # girar primero, sin empujar

        # differential > 0 → M2 sube, M1 baja
        m1 = clamp(thrust - differential, 0, 100)
        m2 = clamp(thrust + differential, 0, 100)
        return int(m1), int(m2)


# ══════════════════════════════════════════════════════════════════════
#  NODO ROS2 PRINCIPAL
# ══════════════════════════════════════════════════════════════════════

class SpaceshipController(Node):

    STRATEGIES = {
        'bangbang': BangBangController,
        'pid':      PIDController,
        'fuzzy':    FuzzyController,
    }

    def __init__(self):
        super().__init__('spaceship_controller')

        self.declare_parameter('target_x',  _NO_TARGET)
        self.declare_parameter('target_y',  _NO_TARGET)
        self.declare_parameter('strategy',  'pid')

        param_x  = self.get_parameter('target_x').value
        param_y  = self.get_parameter('target_y').value
        strategy = self.get_parameter('strategy').value.lower()

        if strategy not in self.STRATEGIES:
            self.get_logger().warn(
                f'Estrategia "{strategy}" desconocida. Usando "pid".'
            )
            strategy = 'pid'

        self.ctrl = self.STRATEGIES[strategy]()
        self.get_logger().info(f'Estrategia seleccionada: {strategy.upper()}')

        self.state:    ShipState | None = None
        self.target_x: float | None     = None
        self.target_y: float | None     = None
        self._dt = 0.05   # 20 Hz

        if param_x != _NO_TARGET and param_y != _NO_TARGET:
            self._set_target(param_x, param_y, 'parámetro launch')

        self.create_subscription(ShipState,    '/ship_state',    self.on_ship_state,    10)
        self.create_subscription(PointStamped, '/clicked_point', self.on_clicked_point, 10)
        self.pub_motor = self.create_publisher(MotorCommand, '/motor_command', 10)

        self.create_timer(self._dt, self.control_loop)

        status = (f'target=({self.target_x:.1f},{self.target_y:.1f})'
                  if self.target_x is not None else 'esperando target (click en RViz)')
        self.get_logger().info(f'🚀 Controlador listo — {status}')

    def _set_target(self, x: float, y: float, source: str = ''):
        self.target_x = x
        self.target_y = y
        self.ctrl.reset()
        self.get_logger().info(
            f'🎯 Target: ({x:.2f}, {y:.2f})'
            + (f'  [{source}]' if source else '')
        )

    def on_ship_state(self, msg: ShipState):
        self.state = msg

    def on_clicked_point(self, msg: PointStamped):
        self._set_target(msg.point.x, msg.point.y, 'click RViz')

    def control_loop(self):
        if self.state is None or self.target_x is None:
            return
        if self.state.arrived:
            self._set_motors(0, 0)
            return

        if isinstance(self.ctrl, (PIDController, FuzzyController)):
            m1, m2 = self.ctrl.compute(
                self.state, self.target_x, self.target_y, self._dt)
        else:
            m1, m2 = self.ctrl.compute(
                self.state, self.target_x, self.target_y)

        self._set_motors(m1, m2)

    def _set_motors(self, m1: int, m2: int):
        for motor_id, power in [(1, m1), (2, m2)]:
            cmd          = MotorCommand()
            cmd.motor_id = motor_id
            cmd.power    = max(0, min(100, int(power)))
            self.pub_motor.publish(cmd)

    def _stop(self):
        self._set_motors(0, 0)


# ══════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = SpaceshipController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
