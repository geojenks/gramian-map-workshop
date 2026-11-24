import Sofa.Core
from Sofa.constants import Key


class ParametricGripperController(Sofa.Core.Controller):
    """
    Pressure controller for an arbitrary number of pneunet fingers.

    Keys:
      +/-   inflate/deflate the active target
      0     target = ALL fingers (default)
      1..9  target = that finger only (1-based; 1 = Finger1)

    Kwargs:
      node           required, the root node
      numFingers     required, int
      pressureMin    default 0.0
      pressureMax    default 1.5      (units depend on the scene)
      increment      default 0.01
      signFlip       default False    set True if your cavity STL normals
                                      point such that NEGATIVE value inflates;
                                      the controller will negate internally so
                                      "+" still means "more inflation"
    """

    def __init__(self, *a, **kw):
        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        self.numFingers = kw["numFingers"]
        self.pressureMin = kw.get("pressureMin", 0.0)
        self.pressureMax = kw.get("pressureMax", 1.5)
        self.increment = kw.get("increment", 0.01)
        self.signFlip = kw.get("signFlip", False)
        # Cavity-less CADs (cable finger) have no pressure constraint to
        # drive; the controller then becomes inert (headless cable sweeps
        # drive the CableConstraints directly).
        self.constraints = []
        for i in range(self.numFingers):
            finger = self.node.getChild('Finger' + str(i + 1))
            cavity = finger.getChild('Cavity')
            if cavity is not None:
                self.constraints.append(cavity.SurfacePressureConstraint)
        if not self.constraints:
            print("[ParametricGripperController] no pressure chambers — "
                  "keyboard pressure control disabled")
        self.target = None  # None means "all"

    def _targets(self):
        if self.target is None:
            return range(self.numFingers)
        return [self.target]

    def _bump(self, delta):
        if not self.constraints:
            return
        sign = -1.0 if self.signFlip else 1.0
        for i in self._targets():
            # Read current (signed) value back through the sign convention so
            # the user-facing "p" is always non-negative when signFlip=True.
            p = sign * self.constraints[i].value.value[0] + delta
            p = max(self.pressureMin, min(self.pressureMax, p))
            self.constraints[i].value = [sign * p]

    def onKeypressedEvent(self, e):
        k = e["key"]

        if k == Key.plus:
            self._bump(self.increment)
        elif k == Key.minus:
            self._bump(-self.increment)
        elif k == "0":
            self.target = None
            print("[ParametricGripperController] target = ALL")
        elif k in [str(d) for d in range(1, 10)]:
            idx = int(k) - 1
            if idx < self.numFingers:
                self.target = idx
                print(f"[ParametricGripperController] target = Finger{idx + 1}")
            else:
                print(f"[ParametricGripperController] no Finger{idx + 1} (only {self.numFingers} fingers)")
