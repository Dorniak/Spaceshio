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
╚══════════════════════════════════════════════════════════════════════╝
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from spaceship_msgs.msg import MotorCommand, ShipState

_NO_TARGET = -999.0


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


# ══════════════════════════════════════════════════════════════════════
#  ESTRATEGIA 1: BANG-BANG CON CORRECCIÓN PROPORCIONAL
# ══════════════════════════════════════════════════════════════════════

class BangBangController:
    """
    Máquina de estados clásica con corrección proporcional de heading
    durante la fase de impulso.

    Fases:
        IDLE    → sin target
        ORIENT  → girar hasta apuntar al target
        THRUST  → impulsar corrigiendo heading en vuelo
        BRAKE   → orientar 180° y frenar
        FINE    → ajuste suave final
    """

    # ── Parámetros ────────────────────────────────────────────────────
    ANGLE_TOL           = 0.12   # rad — tolerancia de orientación (~7°)
    BRAKE_DISTANCE      = 5.0    # m — distancia de inicio de frenado
    FINE_DISTANCE       = 2.5    # m — zona de ajuste fino
    POWER_MAX           = 100    # % — potencia máxima
    POWER_BRAKE         = 95     # % — potencia de frenado
    POWER_FINE          = 35     # % — potencia en ajuste fino
    POWER_TURN          = 80     # % — potencia de giro
    KP_HEADING          = 50.0   # ganancia proporcional heading en vuelo

    def __init__(self):
        self.phase = 'IDLE'

    def reset(self):
        self.phase = 'ORIENT'

    def compute(self, s: ShipState, tx: float, ty: float) -> tuple[int, int]:
        """Devuelve (power_m1, power_m2)."""
        dx    = tx - s.x
        dy    = ty - s.y
        dist  = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx)
        herr  = angle_diff(angle, s.heading)
        speed = math.sqrt(s.vx**2 + s.vy**2)

        if self.phase == 'ORIENT':
            if abs(herr) < self.ANGLE_TOL:
                self.phase = 'THRUST'
                return self.POWER_MAX, self.POWER_MAX
            # Girar en la dirección correcta
            if herr > 0:   # target a la izquierda → activar M2
                return 0, self.POWER_TURN
            else:           # target a la derecha  → activar M1
                return self.POWER_TURN, 0

        elif self.phase == 'THRUST':
            if dist < self.BRAKE_DISTANCE:
                self.phase = 'BRAKE'
                return 0, 0

            # Corrección proporcional de heading durante el vuelo
            corr  = clamp(herr * self.KP_HEADING, -self.POWER_MAX, self.POWER_MAX)
            m1 = clamp(self.POWER_MAX - max(0,  corr), 0, self.POWER_MAX)
            m2 = clamp(self.POWER_MAX + min(0, corr), 0, self.POWER_MAX)
            return int(m1), int(m2)

        elif self.phase == 'BRAKE':
            if dist < self.FINE_DISTANCE:
                self.phase = 'FINE'
                return 0, 0

            # Orientar 180° (apuntar hacia atrás para frenar)
            brake_angle = math.atan2(-dy, -dx)
            berr = angle_diff(brake_angle, s.heading)
            if abs(berr) < self.ANGLE_TOL:
                return self.POWER_BRAKE, self.POWER_BRAKE
            if berr > 0:
                return 0, self.POWER_TURN
            else:
                return self.POWER_TURN, 0

        elif self.phase == 'FINE':
            if speed < 0.05 and dist < 2.0:
                return 0, 0

            # Reorientar al target y empujar suave
            if abs(herr) > self.ANGLE_TOL * 1.5:
                if herr > 0:
                    return 0, int(self.POWER_FINE * 0.6)
                else:
                    return int(self.POWER_FINE * 0.6), 0
            corr = clamp(herr * self.KP_HEADING * 0.5, -self.POWER_FINE, self.POWER_FINE)
            m1 = clamp(self.POWER_FINE - max(0,  corr), 0, self.POWER_FINE)
            m2 = clamp(self.POWER_FINE + min(0, corr), 0, self.POWER_FINE)
            return int(m1), int(m2)

        return 0, 0   # IDLE o fallback


# ══════════════════════════════════════════════════════════════════════
#  ESTRATEGIA 2: PID
# ══════════════════════════════════════════════════════════════════════

class PIDController:
    """
    Controlador PID dual:
        · PID de heading   → diferencial de motores (giro)
        · PID de velocidad → potencia base (propulsión)

    Frenado predictivo:
        d_stop = v² / (2 * a_decel_estimate)
        Cuando dist ≤ d_stop, se invierte el heading y se frena.

    Anti-windup:
        El integrador se satura a ±integral_limit para evitar
        acumulación en saturación.
    """

    # ── Parámetros PID heading ────────────────────────────────────────
    KP_H  = 70.0
    KI_H  = 2.0
    KD_H  = 8.0
    I_LIM_H = 40.0    # límite del integrador de heading

    # ── Parámetros PID velocidad ──────────────────────────────────────
    KP_V  = 18.0
    KI_V  = 0.5
    KD_V  = 3.0
    I_LIM_V = 30.0

    # ── Parámetros de frenado predictivo ─────────────────────────────
    A_DECEL_EST   = 2.2    # aceleración de frenado estimada (m/s²)
    BRAKE_MARGIN  = 1.4    # factor de seguridad (activar antes)
    FINE_DIST     = 2.5    # m — zona de aproximación final
    POWER_BASE    = 85     # % — potencia de crucero
    POWER_FINE    = 30     # % — potencia en zona fina
    ANGLE_TOL     = 0.10   # rad

    def __init__(self):
        self._reset_pid()
        self.phase = 'IDLE'

    def _reset_pid(self):
        self.int_h   = 0.0
        self.int_v   = 0.0
        self.prev_h  = 0.0
        self.prev_v  = 0.0

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

        # Distancia de frenado predictiva
        d_stop = (speed**2) / (2 * self.A_DECEL_EST) * self.BRAKE_MARGIN

        if self.phase == 'ORIENT':
            if abs(herr) < self.ANGLE_TOL:
                self.phase = 'THRUST'
                self._reset_pid()
            return self._pid_turn(herr, dt, base_power=0)

        elif self.phase == 'THRUST':
            if dist < self.FINE_DIST:
                self.phase = 'FINE'
                self._reset_pid()
                return 0, 0
            if dist <= d_stop:
                self.phase = 'BRAKE'
                self._reset_pid()
                return 0, 0
            return self._pid_thrust(herr, dt, self.POWER_BASE)

        elif self.phase == 'BRAKE':
            if dist < self.FINE_DIST or speed < 0.3:
                self.phase = 'FINE'
                self._reset_pid()
                return 0, 0
            # Apuntar 180° y frenar a fondo
            brake_angle = math.atan2(-dy, -dx)
            berr = angle_diff(brake_angle, s.heading)
            if abs(berr) < self.ANGLE_TOL * 1.5:
                return self._pid_thrust(berr, dt, 95)
            return self._pid_turn(berr, dt, base_power=0)

        elif self.phase == 'FINE':
            if speed < 0.04 and dist < 2.0:
                return 0, 0
            return self._pid_thrust(herr, dt, self.POWER_FINE)

        return 0, 0

    def _pid_turn(self, herr: float, dt: float,
                  base_power: int) -> tuple[int, int]:
        """PID puro de heading: diferencial de motores.
        output > 0 → target a la izquierda → M2 sube, M1 baja.
        """
        self.int_h = clamp(self.int_h + herr * dt, -self.I_LIM_H, self.I_LIM_H)
        deriv = (herr - self.prev_h) / max(dt, 1e-6)
        self.prev_h = herr
        output = (self.KP_H * herr
                  + self.KI_H * self.int_h
                  + self.KD_H * deriv)
        output = clamp(output, -100, 100)
        # Fórmula diferencial simétrica:
        # output > 0 → M2 aumenta, M1 disminuye (giro izquierda)
        # output < 0 → M1 aumenta, M2 disminuye (giro derecha)
        m1 = clamp(base_power - output, 0, 100)
        m2 = clamp(base_power + output, 0, 100)
        return int(m1), int(m2)

    def _pid_thrust(self, herr: float, dt: float,
                    base_power: int) -> tuple[int, int]:
        """Avance con corrección PID de heading."""
        self.int_h = clamp(self.int_h + herr * dt, -self.I_LIM_H, self.I_LIM_H)
        deriv = (herr - self.prev_h) / max(dt, 1e-6)
        self.prev_h = herr
        corr = (self.KP_H * herr
                + self.KI_H * self.int_h
                + self.KD_H * deriv)
        corr = clamp(corr, -base_power, base_power)
        m1 = clamp(base_power - max(0,  corr), 0, 100)
        m2 = clamp(base_power + min(0, corr), 0, 100)
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
        · differential   (-100..100): diferencia M2-M1 (positivo → gira izq)
        · thrust         (0..100):    potencia base

    Reglas (extracto de las más importantes):
        IF heading_error=ZERO  AND dist=FAR   AND speed=SLOW  → thrust=HIGH,   diff=ZERO
        IF heading_error=ZERO  AND dist=FAR   AND speed=FAST  → thrust=MEDIUM, diff=ZERO
        IF heading_error=POSBIG AND dist=ANY  AND speed=ANY   → thrust=LOW,    diff=POSBIG
        IF heading_error=ZERO  AND dist=CLOSE AND speed=FAST  → thrust=ZERO,   diff=ZERO  (frenar)
        ...
    """

    def __init__(self):
        self.phase = 'IDLE'

    def reset(self):
        self.phase = 'RUN'

    # ── Funciones de pertenencia triangulares ─────────────────────────

    @staticmethod
    def _tri(x, a, b, c) -> float:
        """Función de pertenencia triangular [a,b,c]."""
        if x <= a or x >= c:
            return 0.0
        if x <= b:
            return (x - a) / (b - a) if b != a else 1.0
        return (c - x) / (c - b) if c != b else 1.0

    @staticmethod
    def _trap(x, a, b, c, d) -> float:
        """Función de pertenencia trapezoidal [a,b,c,d]."""
        if x <= a or x >= d:
            return 0.0
        if x <= b:
            return (x - a) / (b - a) if b != a else 1.0
        if x <= c:
            return 1.0
        return (d - x) / (d - c) if d != c else 1.0

    def _fuzzify_heading(self, herr: float) -> dict:
        h = herr  # rad
        return {
            'NEGBIG':   self._trap(h, -math.pi, -math.pi, -0.6, -0.2),
            'NEGSMALL': self._tri(h, -0.5, -0.15, 0.0),
            'ZERO':     self._tri(h, -0.2,  0.0,  0.2),
            'POSSMALL': self._tri(h,  0.0,  0.15, 0.5),
            'POSBIG':   self._trap(h,  0.2,  0.6, math.pi, math.pi),
        }

    def _fuzzify_dist(self, dist: float) -> dict:
        return {
            'CLOSE':  self._trap(dist, 0, 0, 2.0, 4.5),
            'MEDIUM': self._tri(dist,  3.0, 6.0, 10.0),
            'FAR':    self._trap(dist, 7.0, 12.0, 999, 999),
        }

    def _fuzzify_speed(self, speed: float) -> dict:
        return {
            'SLOW':     self._trap(speed, 0, 0, 0.5, 1.5),
            'MEDIUM_S': self._tri(speed,  1.0, 2.5, 4.5),
            'FAST':     self._trap(speed, 3.5, 6.0, 999, 999),
        }

    def _defuzz_centroid(self, rules: list[tuple[float, float]],
                         lo: float, hi: float) -> float:
        """Defuzzificación por centroide discreto."""
        num, den = 0.0, 0.0
        steps = 50
        step  = (hi - lo) / steps
        for i in range(steps + 1):
            x = lo + i * step
            mu = max(min(strength, 1.0) for strength, center in rules
                     if abs(x - center) <= (hi - lo) / len(rules) + step)
            num += mu * x
            den += mu
        return num / den if den > 1e-9 else (lo + hi) / 2

    def compute(self, s: ShipState, tx: float, ty: float,
                dt: float = 0.05) -> tuple[int, int]:
        dx    = tx - s.x
        dy    = ty - s.y
        dist  = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx)
        herr  = angle_diff(angle, s.heading)
        speed = math.sqrt(s.vx**2 + s.vy**2)

        fh = self._fuzzify_heading(herr)
        fd = self._fuzzify_dist(dist)
        fs = self._fuzzify_speed(speed)

        # ── Reglas de inferencia ──────────────────────────────────────
        # Cada regla: (strength, output_value)
        # differential: -100=giro_derecha .. 0=recto .. +100=giro_izquierda
        # thrust:        0=apagado .. 100=máxima potencia

        diff_rules: list[tuple[float, float]] = [
            # SI heading_error NEGBIG  → gira derecha fuerte
            (fh['NEGBIG'],   -85.0),
            # SI heading_error NEGSMALL → gira derecha suave
            (fh['NEGSMALL'], -30.0),
            # SI heading_error ZERO    → recto
            (fh['ZERO'],       0.0),
            # SI heading_error POSSMALL → gira izquierda suave
            (fh['POSSMALL'],  30.0),
            # SI heading_error POSBIG  → gira izquierda fuerte
            (fh['POSBIG'],    85.0),
        ]

        # Thrust: modulado por distancia, velocidad y error de heading
        alineado     = fh['ZERO'] + 0.5 * (fh['POSSMALL'] + fh['NEGSMALL'])
        no_alineado  = fh['POSBIG'] + fh['NEGBIG']

        thrust_rules: list[tuple[float, float]] = [
            # Lejos + lento + alineado → thrust máximo
            (min(fd['FAR'],   fs['SLOW'],     alineado),     95.0),
            # Lejos + rápido + alineado → mantener
            (min(fd['FAR'],   fs['MEDIUM_S'], alineado),     75.0),
            # Lejos + muy rápido → reducir
            (min(fd['FAR'],   fs['FAST'],     alineado),     50.0),
            # Medio + lento + alineado → impulsar
            (min(fd['MEDIUM'],fs['SLOW'],     alineado),     70.0),
            # Medio + rápido → reducir
            (min(fd['MEDIUM'],fs['FAST'],                1), 20.0),
            # Cerca + rápido → frenar (thrust negativo no existe → 0)
            (min(fd['CLOSE'], fs['FAST'],                1),  0.0),
            # Cerca + lento + alineado → empuje suave
            (min(fd['CLOSE'], fs['SLOW'],     alineado),     25.0),
            # No alineado → reducir thrust para girar antes
            (min(no_alineado,                            1), 10.0),
        ]

        # ── Defuzzificación por máximo ponderado (WM) ─────────────────
        def wm(rules):
            num = sum(s * v for s, v in rules)
            den = sum(s     for s, _ in rules)
            return num / den if den > 1e-9 else 0.0

        differential = wm(diff_rules)   # -100..100
        thrust       = wm(thrust_rules) #    0..100

        # Calcular M1, M2 desde thrust y differential
        # Fórmula diferencial simétrica:
        #   differential > 0 → giro izquierda → M2 sube, M1 baja
        #   differential < 0 → giro derecha   → M1 sube, M2 baja
        base = thrust
        m1 = clamp(base - differential, 0, 100)
        m2 = clamp(base + differential, 0, 100)

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

        # ── Parámetros ────────────────────────────────────────────────
        self.declare_parameter('target_x',  _NO_TARGET)
        self.declare_parameter('target_y',  _NO_TARGET)
        self.declare_parameter('strategy',  'pid')   # bangbang | pid | fuzzy

        param_x    = self.get_parameter('target_x').value
        param_y    = self.get_parameter('target_y').value
        strategy   = self.get_parameter('strategy').value.lower()

        # ── Controlador activo ────────────────────────────────────────
        if strategy not in self.STRATEGIES:
            self.get_logger().warn(
                f'Estrategia "{strategy}" desconocida. Usando "pid".'
            )
            strategy = 'pid'

        self.ctrl = self.STRATEGIES[strategy]()
        self.get_logger().info(f'Estrategia seleccionada: {strategy.upper()}')

        # ── Estado ────────────────────────────────────────────────────
        self.state:    ShipState | None = None
        self.target_x: float | None     = None
        self.target_y: float | None     = None
        self._dt = 0.05   # 20 Hz

        if param_x != _NO_TARGET and param_y != _NO_TARGET:
            self._set_target(param_x, param_y, 'parámetro launch')

        # ── Topics ────────────────────────────────────────────────────
        self.create_subscription(ShipState,    '/ship_state',    self.on_ship_state,    10)
        self.create_subscription(PointStamped, '/clicked_point', self.on_clicked_point, 10)
        self.pub_motor = self.create_publisher(MotorCommand, '/motor_command', 10)

        self.create_timer(self._dt, self.control_loop)

        status = (f'target=({self.target_x:.1f},{self.target_y:.1f})'
                  if self.target_x is not None else 'esperando target (click en RViz)')
        self.get_logger().info(f'🚀 Controlador listo — {status}')

    # ── Target ────────────────────────────────────────────────────────

    def _set_target(self, x: float, y: float, source: str = ''):
        self.target_x = x
        self.target_y = y
        self.ctrl.reset()
        self.get_logger().info(
            f'🎯 Target: ({x:.2f}, {y:.2f})'
            + (f'  [{source}]' if source else '')
        )

    # ── Callbacks ─────────────────────────────────────────────────────

    def on_ship_state(self, msg: ShipState):
        self.state = msg

    def on_clicked_point(self, msg: PointStamped):
        self._set_target(msg.point.x, msg.point.y, 'click RViz')

    # ── Loop ──────────────────────────────────────────────────────────

    def control_loop(self):
        if self.state is None or self.target_x is None:
            return
        if self.state.arrived:
            self._set_motors(0, 0)
            return

        # Llamada al controlador activo
        if isinstance(self.ctrl, (PIDController, FuzzyController)):
            m1, m2 = self.ctrl.compute(
                self.state, self.target_x, self.target_y, self._dt)
        else:
            m1, m2 = self.ctrl.compute(
                self.state, self.target_x, self.target_y)

        self._set_motors(m1, m2)

    # ── Helpers ───────────────────────────────────────────────────────

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
