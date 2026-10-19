"""Test-only deterministic source trajectory through production external-RC wiring."""
import json
from pathlib import Path
import sys
from uuid import uuid4

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / 'bullet_drone'))
from bullet_drone.models import DroneState, DroneObservation, SensorReadings, BarometerReading
from bullet_drone.rc import RcCodec
import examples.external_rc.main as runtime


TIMESTEP_S = 1 / 240
HOLD_STEPS = 120
TRAVEL_STEPS = 24


class DropFirstCollision(runtime.ScenarioTermination):
    """Deliberately discard the first proposal to exercise the real retry path."""
    def reset(self):
        super().reset()
        self.discarded = False

    def accept(self, report, **kwargs):
        if '--reject-collisions' in sys.argv:
            return False
        if not self.discarded:
            self.discarded = True
            return False
        return super().accept(report, **kwargs)


class TrajectorySimulation:
    def __init__(self, config, settings, vehicle):
        self.codec = RcCodec(len(vehicle.rotor_positions_m) * vehicle.max_thrust_per_rotor_n, settings.max_rates_rad_s, config.rc)
        self.points = json.loads(Path(sys.argv[2]).read_text())
        self.last_command = None

    def reset(self):
        self.episode = uuid4().hex
        self.step_index = 0
        self.last_command = None
        return self._observation()

    def step(self, channels):
        self.step_index += 1
        self.last_command = self.codec.decode(channels, self.episode, self.step_index)
        return self._observation()

    def _observation(self):
        fraction = max(0., min(1., (self.step_index - HOLD_STEPS) / TRAVEL_STEPS))
        position = tuple(a + fraction * (b - a) for a, b in zip(*self.points))
        simtime = self.step_index * TIMESTEP_S
        state = DroneState(self.episode, self.step_index, simtime, position, (0., 0., 0., 1.), (0., 0., 0.), (0., 0., 0.))
        barometer = BarometerReading(self.episode, self.step_index, simtime, 101325., position[2])
        return DroneObservation(state, SensorReadings(barometer=barometer))

    def close(self):
        pass


if __name__ == '__main__':
    try:
        runtime.ScenarioTermination = DropFirstCollision
        runtime.run(Path(sys.argv[1]), simulation_factory=TrajectorySimulation)
    except KeyboardInterrupt:
        pass
