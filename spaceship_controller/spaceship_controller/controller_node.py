#!/usr/bin/env python3
"""
SPACESHIP CONTROLLER — v3
=========================
Correcciones clave respecto a versiones anteriores:

1. control_loop NUNCA para los motores por state.arrived.
   El simulador marca arrived=True cuando llega, pero el viento puede
   sacar la nave de la zona. Los controladores mantienen hover activo.

2. Cada controlador tiene máquina de estados clara:
     ORIENT → THRUST → BRAKE → HOVER
   HOVER es un estado permanente con corrección activa.

3. HOVER usa un PID posicional en coordenadas del mundo:
   - Calcula el error de posición (dx, dy) respecto al target.
   - Convierte ese error a un heading deseado y empuje.
   - Frena si la velocidad es alta, corrige si se ha desplazado.
   Esto compensa el viento de forma continua.

Física del simulador (referencia):
   ft     = (power_m1 + power_m2) / 2 / 100 * MAX_THRUST   (empuje)
   torque = (power_m1 - power_m2) / 100 * MAX_THRUST * ARM  (giro)
   torque > 0  →  heading sube  →  gira IZQUIERDA
   → herr > 0 (target izquierda) → M1 sube, M2 baja
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from spaceship_msgs.msg import MotorCommand, ShipState

_NO_TARGET   = -999.0
MAX_THRUST   = 3.0
ARM_LENGTH   = 0.7
LINEAR_DRAG  = 0.5
ANGULAR_DRAG = 1.2
INERTIA      = 1.5


# ─────────────────────────────────────────────────────────────────────
# Utilidades
# ─────────────────────────────────────────────────────────────────────

def adiff(a: float, b: float) -> float:
    d = (a - b + math.pi) % (2 * math.pi) - math.pi
    return d


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def brake_dist(vr: float, margin: float = 1.3) -> float:
    """Distancia mínima necesaria para frenar desde velocidad radial vr."""
    if vr <= 0:
        return 0.0
    a_eff = MAX_THRUST - LINEAR_DRAG * vr
    if a_eff <= 0:
        a_eff = 0.3
    return vr * vr / (2.0 * a_eff) * margin


# ─────────────────────────────────────────────────────────────────────
# HOVER — control posicional compartido por los 3 controladores
#
# Objetivo: mantener la nave quieta sobre (tx, ty) con viento.
#
# Estrategia:
#   A) Si speed > VMAX → frenar (apuntar opuesto a velocidad y empujar)
#   B) Si dist > DBAND → corregir (apuntar al target y empujar poco)
#   C) Si dentro de zona muerta → esperar (0,0)
#
# El empuje de corrección es proporcional a la distancia (P-control
# posicional) lo que da un comportamiento de "muelle amortiguado":
# fuerza grande lejos, pequeña cerca, nunca overshoots grandes.
# ─────────────────────────────────────────────────────────────────────

HOVER_VMAX   = 0.30    # m/s  — por encima: frenado activo
HOVER_DBAND  = 0.70    # m    — por debajo: zona muerta (sin empuje)
HOVER_ATOL   = 0.20    # rad  — tolerancia angular para empujar (más permisiva)
HOVER_PBRAKE = 70      # %    — potencia de frenado (subida para frenar más rápido)
HOVER_PCORR  = 28      # %    — potencia máxima de corrección
HOVER_PTURN  = 35      # %    — potencia de giro en hover


def _turn_hover(herr: float) -> tuple:
    """Giro suave en hover. herr>0 → izq → M1 sube."""
    p = int(clamp(abs(herr) / (math.pi / 2) * HOVER_PTURN, 8, HOVER_PTURN))
    return (p, 0) if herr > 0 else (0, p)


def hover(s: ShipState, tx: float, ty: float) -> tuple:
    dx    = tx - s.x
    dy    = ty - s.y
    dist  = math.sqrt(dx * dx + dy * dy)
    speed = math.sqrt(s.vx * s.vx + s.vy * s.vy)

    # ── A: freno activo ───────────────────────────────────────────────
    if speed > HOVER_VMAX:
        brake_hdg = math.atan2(-s.vy, -s.vx)       # opuesto a velocidad
        berr = adiff(brake_hdg, s.heading)
        if abs(berr) < HOVER_ATOL:
            return HOVER_PBRAKE, HOVER_PBRAKE
        return _turn_hover(berr)

    # ── B: corrección posicional ──────────────────────────────────────
    if dist > HOVER_DBAND:
        herr = adiff(math.atan2(dy, dx), s.heading)
        if abs(herr) < HOVER_ATOL:
            # Empuje proporcional a distancia (P-control)
            pwr = int(clamp(dist / 5.0 * HOVER_PCORR, 6, HOVER_PCORR))
            return pwr, pwr
        return _turn_hover(herr)

    # ── C: zona muerta ────────────────────────────────────────────────
    return 0, 0


# ─────────────────────────────────────────────────────────────────────
# ESTRATEGIA 1 — BANG-BANG
# ─────────────────────────────────────────────────────────────────────

class BangBangController:
    ATOL   = 0.10
    PTURN  = 60
    PMAX   = 80
    PBRAKE = 80
    KPH    = 45.0
    BMARGIN = 1.3
    BTOL   = 0.20

    def __init__(self): self.phase = 'IDLE'
    def reset(self):    self.phase = 'ORIENT'

    def compute(self, s: ShipState, tx: float, ty: float) -> tuple:
        dx = tx - s.x;  dy = ty - s.y
        dist  = math.sqrt(dx*dx + dy*dy)
        herr  = adiff(math.atan2(dy, dx), s.heading)
        speed = math.sqrt(s.vx*s.vx + s.vy*s.vy)
        vr    = (s.vx*dx + s.vy*dy) / max(dist, 0.01)

        # ── Entrada a HOVER ───────────────────────────────────────────
        # Condición: dist pequeña O velocidad hacia el target y frenado necesario
        need_brake = brake_dist(max(vr, 0)) >= dist and vr > 0.2
        if (dist < 3.0 and speed < 1.5) or (dist < 1.5):
            self.phase = 'HOVER'

        if self.phase == 'ORIENT':
            if abs(herr) < self.ATOL:
                self.phase = 'THRUST'
            return self._turn(herr)

        elif self.phase == 'THRUST':
            if need_brake:
                self.phase = 'BRAKE'
                return 0, 0
            corr = clamp(herr * self.KPH, -self.PMAX, self.PMAX)
            return int(clamp(self.PMAX + corr, 0, self.PMAX)), \
                   int(clamp(self.PMAX - corr, 0, self.PMAX))

        elif self.phase == 'BRAKE':
            if speed < 0.25:
                self.phase = 'HOVER'
                return 0, 0
            brake_hdg = math.atan2(-s.vy, -s.vx)
            berr = adiff(brake_hdg, s.heading)
            if abs(berr) < self.BTOL:
                return self.PBRAKE, self.PBRAKE
            return self._turn(berr)

        elif self.phase == 'HOVER':
            return hover(s, tx, ty)

        return 0, 0

    def _turn(self, herr):
        return (self.PTURN, 0) if herr > 0 else (0, self.PTURN)


# ─────────────────────────────────────────────────────────────────────
# ESTRATEGIA 2 — PID
# ─────────────────────────────────────────────────────────────────────

class PIDController:
    KPH = 55.0;  KIH = 0.8;  KDH = 8.0;  ILIM = 25.0
    BMARGIN = 1.25;  PBASE = 70;  PBRAKE = 85
    ATOL = 0.10;  BTOL = 0.20

    def __init__(self):
        self.ih = 0.0;  self.ph = 0.0;  self.phase = 'IDLE'

    def _rpid(self): self.ih = 0.0; self.ph = 0.0

    def reset(self): self.phase = 'ORIENT'; self._rpid()

    def compute(self, s: ShipState, tx: float, ty: float, dt: float = 0.05) -> tuple:
        dx = tx - s.x;  dy = ty - s.y
        dist  = math.sqrt(dx*dx + dy*dy)
        herr  = adiff(math.atan2(dy, dx), s.heading)
        speed = math.sqrt(s.vx*s.vx + s.vy*s.vy)
        vr    = (s.vx*dx + s.vy*dy) / max(dist, 0.01)
        need_brake = brake_dist(max(vr, 0)) >= dist and vr > 0.2

        # ── Entrada a HOVER ───────────────────────────────────────────
        if (dist < 3.0 and speed < 1.5) or (dist < 1.5):
            if self.phase not in ('HOVER',):
                self.phase = 'HOVER'; self._rpid()

        if self.phase == 'ORIENT':
            if abs(herr) < self.ATOL:
                self.phase = 'THRUST'; self._rpid()
            return self._pturn(herr, dt)

        elif self.phase == 'THRUST':
            if need_brake:
                self.phase = 'BRAKE'; self._rpid(); return 0, 0
            return self._pthrust(herr, dt, self.PBASE)

        elif self.phase == 'BRAKE':
            if speed < 0.25:
                self.phase = 'HOVER'; self._rpid(); return 0, 0
            brake_hdg = math.atan2(-s.vy, -s.vx)
            berr = adiff(brake_hdg, s.heading)
            if abs(berr) < self.BTOL:
                return self.PBRAKE, self.PBRAKE
            return self._pturn(berr, dt)

        elif self.phase == 'HOVER':
            return hover(s, tx, ty)

        return 0, 0

    def _pturn(self, herr, dt):
        self.ih = clamp(self.ih + herr * dt, -self.ILIM, self.ILIM)
        d = (herr - self.ph) / max(dt, 1e-6);  self.ph = herr
        out = clamp(self.KPH*herr + self.KIH*self.ih + self.KDH*d, -100, 100)
        return int(clamp(out, 0, 100)), int(clamp(-out, 0, 100))

    def _pthrust(self, herr, dt, base):
        self.ih = clamp(self.ih + herr * dt, -self.ILIM, self.ILIM)
        d = (herr - self.ph) / max(dt, 1e-6);  self.ph = herr
        c = clamp(self.KPH*herr + self.KIH*self.ih + self.KDH*d, -base, base)
        return int(clamp(base+c, 0, 100)), int(clamp(base-c, 0, 100))


# ─────────────────────────────────────────────────────────────────────
# ESTRATEGIA 3 — FUZZY
# ─────────────────────────────────────────────────────────────────────

class FuzzyController:
    BMARGIN = 1.3

    def __init__(self): self.phase = 'IDLE'
    def reset(self):    self.phase = 'RUN'

    @staticmethod
    def _tri(x, a, b, c):
        if x <= a or x >= c: return 0.0
        return (x-a)/(b-a) if x <= b else (c-x)/(c-b)

    @staticmethod
    def _trap(x, a, b, c, d):
        if x <= a or x >= d: return 0.0
        if x >= b and x <= c: return 1.0
        return (x-a)/(b-a) if x < b else (d-x)/(d-c)

    def _fh(self, h):
        return {
            'NB': self._trap(h, -math.pi, -math.pi, -0.6, -0.2),
            'NS': self._tri(h, -0.5, -0.15, 0.0),
            'ZE': self._tri(h, -0.2, 0.0, 0.2),
            'PS': self._tri(h, 0.0, 0.15, 0.5),
            'PB': self._trap(h, 0.2, 0.6, math.pi, math.pi),
        }

    def _fd(self, d):
        return {
            'C': self._trap(d, 0, 0, 3.0, 6.0),
            'M': self._tri(d, 4.0, 7.0, 11.0),
            'F': self._trap(d, 8.0, 13.0, 999, 999),
        }

    def _fs(self, sp):
        return {
            'S': self._trap(sp, 0, 0, 0.6, 1.5),
            'M': self._tri(sp, 1.0, 2.5, 4.5),
            'F': self._trap(sp, 3.5, 6.0, 999, 999),
        }

    @staticmethod
    def _wm(rules):
        n = sum(w*v for w,v in rules);  d = sum(w for w,_ in rules)
        return n/d if d > 1e-9 else 0.0

    def compute(self, s: ShipState, tx: float, ty: float, dt: float = 0.05) -> tuple:
        dx = tx - s.x;  dy = ty - s.y
        dist  = math.sqrt(dx*dx + dy*dy)
        herr  = adiff(math.atan2(dy, dx), s.heading)
        speed = math.sqrt(s.vx*s.vx + s.vy*s.vy)
        vr    = (s.vx*dx + s.vy*dy) / max(dist, 0.01)

        # ── Entrada a HOVER ───────────────────────────────────────────
        # Condiciones para entrar en HOVER (control posicional preciso):
        #   1. Cerca y velocidad baja (condicion clasica)
        #   2. Muy cerca, sin importar velocidad
        #   3. La distancia de frenado ya alcanza la distancia al target:
        #      hay que frenar YA o nos pasamos del target
        #   4. Overshoot: pasamos el target y nos alejamos de el
        need_b   = brake_dist(max(vr, 0), self.BMARGIN * 1.4)
        overshot  = (dist < 6.0 and vr < -0.25)
        must_brake = (vr > 0.2 and need_b >= dist * 0.85)
        if (dist < 3.0 and speed < 1.5) or (dist < 1.5) or must_brake or overshot:
            self.phase = 'HOVER'

        if self.phase == 'HOVER':
            return hover(s, tx, ty)

        # ── Lógica fuzzy ──────────────────────────────────────────────
        fh = self._fh(herr);  fd = self._fd(dist);  fs = self._fs(speed)
        ali = fh['ZE'] + 0.5*(fh['PS']+fh['NS'])
        nali = fh['PB'] + fh['NB']

        diff = self._wm([
            (fh['NB'], -85.0), (fh['NS'], -30.0), (fh['ZE'], 0.0),
            (fh['PS'],  30.0), (fh['PB'],  85.0),
        ])

        need_b = brake_dist(max(vr,0), self.BMARGIN)
        nb = clamp((need_b - dist) / max(dist*0.3, 1.0), 0, 1) if vr > 0.2 else 0.0

        thrust = self._wm([
            (min(fd['F'], fs['S'], ali),  75.0),
            (min(fd['F'], fs['M'], ali),  55.0),
            (min(fd['F'], fs['F'], ali),  30.0),
            (min(fd['M'], fs['S'], ali),  50.0),
            (min(fd['M'], fs['M'], ali),  20.0),
            (min(fd['M'], fs['F'],    1),  5.0),
            (min(fd['C'], fs['F'],    1),  0.0),
            (min(fd['C'], fs['S'], ali),  10.0),
            (min(nali,              1),    3.0),
            (nb,                           0.0),
        ])

        # Maniobra de frenado activo
        if nb > 0.5 and speed > 0.5:
            brake_hdg = math.atan2(-s.vy, -s.vx)
            berr = adiff(brake_hdg, s.heading)
            diff = clamp(berr * 80.0, -100, 100)
            thrust = 70.0 if abs(berr) < 0.25 else 0.0

        m1 = int(clamp(thrust + diff, 0, 100))
        m2 = int(clamp(thrust - diff, 0, 100))
        return m1, m2


# ─────────────────────────────────────────────────────────────────────
# NODO ROS2
# ─────────────────────────────────────────────────────────────────────

class SpaceshipController(Node):

    STRATEGIES = {'bangbang': BangBangController,
                  'pid':      PIDController,
                  'fuzzy':    FuzzyController}

    def __init__(self):
        super().__init__('spaceship_controller')
        self.declare_parameter('target_x', _NO_TARGET)
        self.declare_parameter('target_y', _NO_TARGET)
        self.declare_parameter('strategy', 'pid')

        px = self.get_parameter('target_x').value
        py = self.get_parameter('target_y').value
        st = self.get_parameter('strategy').value.lower()
        if st not in self.STRATEGIES:
            self.get_logger().warn(f'Estrategia "{st}" desconocida → pid')
            st = 'pid'

        self.ctrl     = self.STRATEGIES[st]()
        self.state    = None
        self.target_x = None
        self.target_y = None
        self._dt      = 0.05

        self.get_logger().info(f'Estrategia: {st.upper()}')

        if px != _NO_TARGET and py != _NO_TARGET:
            self._set_target(px, py, 'launch param')

        self.create_subscription(ShipState,    '/ship_state',    self._on_state,   10)
        self.create_subscription(PointStamped, '/clicked_point', self._on_click,   10)
        self.pub = self.create_publisher(MotorCommand, '/motor_command', 10)
        self.create_timer(self._dt, self._loop)

    def _set_target(self, x, y, src=''):
        self.target_x = x;  self.target_y = y
        self.ctrl.reset()
        self.get_logger().info(f'🎯 Target ({x:.2f}, {y:.2f}) [{src}]')

    def _on_state(self, msg): self.state = msg
    def _on_click(self, msg): self._set_target(msg.point.x, msg.point.y, 'RViz click')

    def _loop(self):
        if self.state is None or self.target_x is None:
            return

        # ── CRÍTICO: NO parar por state.arrived ──────────────────────
        # El simulador marca arrived=True al llegar pero el viento puede
        # sacar la nave de la zona. El controlador siempre sigue activo.

        if isinstance(self.ctrl, (PIDController, FuzzyController)):
            m1, m2 = self.ctrl.compute(self.state, self.target_x, self.target_y, self._dt)
        else:
            m1, m2 = self.ctrl.compute(self.state, self.target_x, self.target_y)

        self._motors(m1, m2)

    def _motors(self, m1, m2):
        for mid, pwr in [(1, m1), (2, m2)]:
            cmd = MotorCommand()
            cmd.motor_id = mid
            cmd.power    = int(clamp(pwr, 0, 100))
            self.pub.publish(cmd)

    def _stop(self): self._motors(0, 0)


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
