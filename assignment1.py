import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['FULL_RECALCULATE'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 30
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference = [50, 0, 0]

    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        v_t = prev_state[3] # m/s

        x_t_1 = x_t + v_t * dt
        v_t_1 = v_t + pedal * dt
        return [x_t_1, 0, 0, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0

        prev_v = state[3]

        for i in range(self.horizon):
            state = self.plant_model(state, self.dt, u[i * 2], 0.0)

            cost += 10 * (state[0] - ref[0]) ** 2

            cost += 1000 * abs(prev_v - state[3])

            prev_v = state[3]

            speed_kph = state[3] * 3.6

            if speed_kph > 10:
                cost += 1000 * speed_kph

        return cost

sim_run(options, ModelPredictiveControl)
