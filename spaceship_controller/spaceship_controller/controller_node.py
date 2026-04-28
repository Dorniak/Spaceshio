#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════╗
║   SPACESHIP CONTROLLER — VERSIÓN CORREGIDA                          ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                      ║
║   FIX CRÍTICO: Convención de motores corregida para coincidir       ║
║   con la implementación REAL del simulador (no su comentario).      ║
║                                                                      ║
║   FÍSICA REAL DEL SIMULADOR (línea 245):                            ║
║     torque = (F1 - F2) * ARM_LENGTH                                 ║
║     F1 > F2 → torque > 0 → heading AUMENTA → gira IZQUIERDA        ║
║     F2 > F1 → torque < 0 → heading DECRECE → gira DERECHA          ║
║                                                                      ║
║   Por tanto:                                                         ║
║     herr > 0 (target izquierda)  → necesito girar IZQ → M1 sube     ║
║     herr < 0 (target derecha)    → necesito girar DER → M2 sube     ║
║                                                                      ║
╚══════════════════════════════════════════════════════════════════════╝
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from spaceship_msgs.msg import MotorCommand, ShipState

_NO_TARGET = -999.0

# ── Física del simulador ──────────────────────────────────────────────
MAX_THRUST   = 3.0
ARM_LENGTH   = 0.7
LINEAR_DRAG  = 0.5
ANGULAR_DRAG = 1.2
INERTIA      = 1.5
A_MAX        = MAX_THRUST


def angle_diff(a: float, b: float) -> float:
    """Diferencia angular normalizada a [-π, π]."""
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def braking_distance(speed: float, margin: float = 1.2) -> float:
    """Distancia de frenado con margen de seguridad."""
    a_eff = max(A_MAX * 0.5, A_MAX - LINEAR_DRAG * speed)
    return (speed * speed) / (2.0 * a_eff) * margin


# ══════════════════════════════════════════════════════════════════════
#  ESTRATEGIA 1: BANG-BANG
# ══════════════════════════════════════════════════════════════════════

class BangBangController:
    """
    Convención CORREGIDA:
        torque = (F1 - F2) * ARM
        herr > 0 → target IZQUIERDA  → girar IZQ → torque > 0 → M1 SUBE
        herr < 0 → target DERECHA    → girar DER → torque < 0 → M2 SUBE
    """

    ANGLE_TOL      = 0.10
    BRAKE_TOL      = 0.20
    POWER_MAX      = 80
    POWER_BRAKE    = 80
    POWER_FINE     = 25
    POWER_TURN     = 60
    KP_HEADING     = 40.0
    BRAKE_MARGIN   = 1.3
    FINE_DISTANCE  = 2.0
    FINE_SPEED     = 0.5

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
        vr    = (s.vx * dx + s.vy * dy) / max(dist, 0.1)
        d_stop = braking_distance(max(vr, 0.0), self.BRAKE_MARGIN)

        if self.phase == 'ORIENT':
            if abs(herr) < self.ANGLE_TOL:
                self.phase = 'THRUST'
                return self.POWER_MAX, self.POWER_MAX
            return self._turn(herr)

        elif self.phase == 'THRUST':
            if dist < self.FINE_DISTANCE:
                self.phase = 'FINE'
                return 0, 0
            if vr > 0.1 and d_stop >= dist:
                self.phase = 'BRAKE'
                return 0, 0
            # Corrección: herr > 0 → M1 sube, M2 baja
            corr = clamp(herr * self.KP_HEADING, -self.POWER_MAX, self.POWER_MAX)
            m1 = clamp(self.POWER_MAX + corr, 0, self.POWER_MAX)
            m2 = clamp(self.POWER_MAX - corr, 0, self.POWER_MAX)
            return int(m1), int(m2)

        elif self.phase == 'BRAKE':
            if dist < self.FINE_DISTANCE:
                self.phase = 'FINE'
                return 0, 0
            if vr < 0.2 and speed < self.FINE_SPEED:
                self.phase = 'FINE'
                return 0, 0
            brake_angle = math.atan2(-dy, -dx)
            berr = angle_diff(brake_angle, s.heading)
            if abs(berr) < self.BRAKE_TOL:
                return self.POWER_BRAKE, self.POWER_BRAKE
            return self._turn(berr)

        elif self.phase == 'FINE':
            if speed < 0.08 and dist < self.FINE_DISTANCE:
                return 0, 0
            if speed > self.FINE_SPEED:
                vel_angle   = math.atan2(s.vy, s.vx)
                brake_angle = vel_angle + math.pi
                berr = angle_diff(brake_angle, s.heading)
                if abs(berr) < self.BRAKE_TOL * 1.5:
                    return self.POWER_FINE, self.POWER_FINE
                return self._turn_fine(berr)
            if dist > 0.5 and abs(herr) > self.ANGLE_TOL * 2:
                return self._turn_fine(herr)
            if dist > 0.5:
                corr = clamp(herr * self.KP_HEADING * 0.3, -self.POWER_FINE, self.POWER_FINE)
                m1 = clamp(self.POWER_FINE + corr, 0, self.POWER_FINE)
                m2 = clamp(self.POWER_FINE - corr, 0, self.POWER_FINE)
                return int(m1), int(m2)
            return 0, 0

        return 0, 0

    def _turn(self, herr: float) -> tuple[int, int]:
        """herr > 0 → izquierda → M1 sube (CORREGIDO)"""
        if herr > 0:
            return self.POWER_TURN, 0
        else:
            return 0, self.POWER_TURN

    def _turn_fine(self, herr: float) -> tuple[int, int]:
        p = int(self.POWER_TURN * 0.5)
        if herr > 0:
            return p, 0
        else:
            return 0, p


# ══════════════════════════════════════════════════════════════════════
#  ESTRATEGIA 2: PID
# ══════════════════════════════════════════════════════════════════════

class PIDController:
    """
    Convención CORREGIDA:
        output > 0 → herr > 0 → target izquierda → M1 sube
    """

    KP_H    = 60.0
    KI_H    = 1.5
    KD_H    = 10.0
    I_LIM_H = 30.0

    BRAKE_MARGIN  = 1.25
    FINE_DIST     = 2.0
    POWER_BASE    = 70
    POWER_BRAKE   = 85
    POWER_FINE    = 22
    ANGLE_TOL     = 0.10
    BRAKE_TOL     = 0.20

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
            vel_angle   = math.atan2(s.vy, s.vx)
            brake_angle = vel_angle + math.pi
            berr = angle_diff(brake_angle, s.heading)
            if abs(berr) < self.BRAKE_TOL:
                return self.POWER_BRAKE, self.POWER_BRAKE
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
        CORREGIDO: output > 0 → herr > 0 → izquierda → M1 sube
        """
        self.int_h = clamp(self.int_h + herr * dt, -self.I_LIM_H, self.I_LIM_H)
        deriv = (herr - self.prev_h) / max(dt, 1e-6)
        self.prev_h = herr
        output = clamp(
            self.KP_H * herr + self.KI_H * self.int_h + self.KD_H * deriv,
            -100, 100
        )
        # CORREGIDO: output > 0 → M1 sube
        m1 = clamp( output, 0, 100)
        m2 = clamp(-output, 0, 100)
        return int(m1), int(m2)

    def _pid_thrust(self, herr: float, dt: float,
                    base_power: int) -> tuple[int, int]:
        """
        CORREGIDO: corr > 0 → herr > 0 → izquierda → M1 sube
        """
        self.int_h = clamp(self.int_h + herr * dt, -self.I_LIM_H, self.I_LIM_H)
        deriv = (herr - self.prev_h) / max(dt, 1e-6)
        self.prev_h = herr
        corr = clamp(
            self.KP_H * herr + self.KI_H * self.int_h + self.KD_H * deriv,
            -base_power, base_power
        )
        # CORREGIDO: corr > 0 → M1 sube
        m1 = clamp(base_power + corr, 0, 100)
        m2 = clamp(base_power - corr, 0, 100)
        return int(m1), int(m2)


# ══════════════════════════════════════════════════════════════════════
#  ESTRATEGIA 3: FUZZY
# ══════════════════════════════════════════════════════════════════════

class FuzzyController:
    """
    Convención CORREGIDA:
        differential > 0 → M1 sube, M2 baja (gira izquierda)
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

        # CORREGIDO: differential > 0 → M1 sube (izquierda)
        # herr > 0 → target izquierda → differential positivo
        diff_rules: list[tuple[float, float]] = [
            (fh['NEGBIG'],   -85.0),   # derecha fuerte → M2 sube
            (fh['NEGSMALL'], -30.0),
            (fh['ZERO'],       0.0),
            (fh['POSSMALL'],  30.0),   # izquierda suave → M1 sube
            (fh['POSBIG'],    85.0),   # izquierda fuerte → M1 sube
        ]

        d_stop = braking_distance(max(vr, 0.0), self.BRAKE_MARGIN)
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

        if need_brake > 0.5 and speed > 0.4:
            vel_angle   = math.atan2(s.vy, s.vx)
            brake_angle = vel_angle + math.pi
            berr = angle_diff(brake_angle, s.heading)
            # CORREGIDO: berr > 0 → izquierda → differential positivo → M1 sube
            differential = clamp(berr * 80.0, -100, 100)
            if abs(berr) < 0.25:
                thrust = 70.0
            else:
                thrust = 0.0

        # CORREGIDO: differential > 0 → M1 sube
        m1 = clamp(thrust + differential, 0, 100)
        m2 = clamp(thrust - differential, 0, 100)
        return int(m1), int(m2)


# ══════════════════════════════════════════════════════════════════════
#  NODO ROS2
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
        self._dt = 0.05

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
