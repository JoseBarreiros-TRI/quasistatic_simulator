import os
import numpy as np

from pydrake.all import PiecewisePolynomial

from examples.setup_simulations import run_quasistatic_sim
from qsim.simulator import QuasistaticSimParameters
from qsim.model_paths import models_dir
from qsim.parser import QuasistaticParser, QuasistaticSystemBackend

model_directive_path = os.path.join(models_dir, "q_sys", "box_pushing.yml")

#%% sim setup

q_parser =QuasistaticParser(model_directive_path)

h = q_parser.get_param_attribute("h")
T = int(round(5.0 / h))  # num of time steps to simulate forward.
duration = T * h
print(duration)

# trajectory and initial conditions.
nq_a = 2
qa_knots = np.zeros((2, nq_a))
# x, y, z, dy1, dy2
qa_knots[0] = [0.0, -0.2]
qa_knots[1] = [0.1, 1.0]
q_robot_traj = PiecewisePolynomial.FirstOrderHold([0, T * h], qa_knots.T)

q_a_traj_dict_str = {"hand": q_robot_traj}
q_u0 = np.array([0, 0.5, 0])
q0_dict_str = {"box": q_u0, "hand": qa_knots[0]}


#%% run sim.
if __name__ == "__main__":
    loggers_dict_quasistatic_str, q_sys = run_quasistatic_sim(
        q_parser = q_parser,
        backend =  QuasistaticSystemBackend.CPP,
        q_a_traj_dict_str=q_a_traj_dict_str,
        q0_dict_str=q0_dict_str,
        is_visualizing=True,
        real_time_rate=1.0,
    )
