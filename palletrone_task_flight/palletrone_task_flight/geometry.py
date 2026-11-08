"""Level-body FRD to local-NED geometry. Angles in radians, lengths in metres."""
from dataclasses import dataclass
from math import cos, sin, atan2, isfinite, sqrt


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def scale(a, k):
    return tuple(k * x for x in a)


def norm(a):
    return sqrt(sum(x*x for x in a))


def rotate(yaw, vector):
    x, y, z = vector
    return (cos(yaw)*x - sin(yaw)*y, sin(yaw)*x + cos(yaw)*y, z)


def direction(theta):
    return (cos(theta), 0.0, sin(theta))


def angle_error(a, b):
    return atan2(sin(a-b), cos(a-b))


@dataclass(frozen=True)
class Geometry:
    length: float = 0.60
    reference_to_bottom: tuple = (0.0, 0.0, 0.0)
    mount_from_bottom: tuple = (-0.11, 0.0, 0.0)

    def __post_init__(self):
        if not isfinite(self.length) or self.length <= 0:
            raise ValueError('rod length must be positive and finite')
        for v in (self.reference_to_bottom, self.mount_from_bottom):
            if len(v) != 3 or not all(isfinite(x) for x in v):
                raise ValueError('FRD offsets require three finite values')

    @property
    def mount(self):
        return add(self.reference_to_bottom, self.mount_from_bottom)

    def points(self, position, yaw, theta):
        mount = add(position, rotate(yaw, self.mount))
        end = add(mount, scale(rotate(yaw, direction(theta)), self.length))
        return mount, end

    def contact(self, position, yaw, theta, distance):
        return add(position, rotate(yaw, add(self.mount, scale(direction(theta), self.length+distance))))

    def target(self, contact, yaw, theta, distance):
        if distance < 0 or not all(isfinite(v) for v in (*contact, yaw, theta, distance)):
            raise ValueError('finite geometry and nonnegative standoff required')
        return sub(contact, rotate(yaw, add(self.mount, scale(direction(theta), self.length+distance))))


def smoothstep(u):
    u = min(1.0, max(0.0, u))
    # Roundoff near u=1 can otherwise make the shared scalar exceed one.
    return min(1.0, max(0.0, u*u*u * (10.0 + u * (-15.0 + 6.0*u))))
