import os
import numpy as np

from pydrake.all import PiecewisePolynomial
from qsim.parser import QuasistaticParser, QuasistaticSystemBackend
from examples.setup_simulations import run_quasistatic_sim
from qsim.simulator import QuasistaticSimParameters
from qsim.model_paths import models_dir

model_directive_path = os.path.join(models_dir, "q_sys", "box_pivoting.yml")

#%% sim setup
h = 0.1
T = int(round(5.0 / h))  # num of time steps to simulate forward.
duration = T * h

q_parser = QuasistaticParser(model_directive_path)
q_parser.set_sim_params(
    h=h,
    log_barrier_weight=100,
    )

# trajectory and initial conditions.
nq_a = 2
qa_knots = np.zeros((2, nq_a))
# x, y, z, dy1, dy2
qa_knots[0] = [-0.7, 0.5]
qa_knots[1] = [1.0, 1.2]
q_robot_traj = PiecewisePolynomial.FirstOrderHold([0, T * h], qa_knots.T)

q_a_traj_dict_str = {"hand": q_robot_traj}

q_u0 = np.array([0, 0.6, 0])

q0_dict_str = {"box": q_u0, "hand": qa_knots[0]}


#%% run sim.
if __name__ == "__main__":
    loggers_dict_quasistatic_str, q_sys = run_quasistatic_sim(
        q_parser=q_parser,
        backend=QuasistaticSystemBackend.CPP,
        q_a_traj_dict_str=q_a_traj_dict_str,
        q0_dict_str=q0_dict_str,
        is_visualizing=True,
        real_time_rate=1.0,
    )
