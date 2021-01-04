import matplotlib.pyplot as plt

from examples.setup_simulation_diagram import *
from examples.setup_environments import create_iiwa_plant
from examples.log_comparison import calc_error_integral
from iiwa_controller.iiwa_controller.utils import create_iiwa_controller_plant

# Simulation parameters.
nq_a = 7
gravity = np.array([0, 0, -10.])
Kp_iiwa = np.array([800., 600, 600, 600, 400, 200, 200])
robot_name = "iiwa7"

qa_knots = np.zeros((2, nq_a))
qa_knots[0] = [0, 0, 0, -1.70, 0, 1.0, 0]
qa_knots[1] = qa_knots[0] + 0.5

q_iiwa_traj = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
    breaks=[0, 10], samples=qa_knots.T,
    sample_dot_at_start=np.zeros(nq_a),
    sample_dot_at_end=np.zeros(nq_a))

q0_dict_str = {robot_name: qa_knots[0]}

h_quasistatic = 0.2
h_mbp = 1e-4


def run_comparison(is_visualizing=False, real_time_rate=0.):
    #%% Quasistatic
    loggers_dict_quasistatic_str, q_sys = run_quasistatic_sim(
        q_a_traj_dict_str={robot_name: q_iiwa_traj},
        q0_dict_str=q0_dict_str,
        Kp_list=[Kp_iiwa],
        setup_environment=create_iiwa_plant,
        object_sdf_paths=[],
        h=h_quasistatic,
        gravity=gravity,
        is_visualizing=is_visualizing,
        real_time_rate=real_time_rate)

    #%% MBP
    loggers_dict_mbp_str = run_mbp_sim(
        q_a_traj=q_iiwa_traj,
        q0_dict_str=q0_dict_str,
        Kp_a=Kp_iiwa,
        object_sdf_paths=[],
        setup_environment=create_iiwa_plant,
        create_controller_plant=create_iiwa_controller_plant,
        h=h_mbp,
        gravity=gravity,
        is_visualizing=is_visualizing,
        real_time_rate=real_time_rate)

    # Extracting iiwa configuration logs.
    q_iiwa_log_mbp = loggers_dict_mbp_str[robot_name].data()[:nq_a].T
    t_mbp = loggers_dict_mbp_str[robot_name].sample_times()

    q_iiwa_log_quasistatic = loggers_dict_quasistatic_str[robot_name].data().T
    t_quasistatic = loggers_dict_quasistatic_str[robot_name].sample_times()

    return q_iiwa_log_mbp, t_mbp, q_iiwa_log_quasistatic, t_quasistatic


if __name__ == "__main__":
    q_iiwa_log_mbp, t_mbp, q_iiwa_log_quasistatic, t_quasistatic = \
        run_comparison(is_visualizing=True, real_time_rate=0.0)

    # Making plots.
    figure, axes = plt.subplots(7, 1, figsize=(4, 10), dpi=200)
    for i, ax in enumerate(axes):
        ax.plot(t_mbp, q_iiwa_log_mbp[:, i])
        ax.plot(t_quasistatic, q_iiwa_log_quasistatic[:, i])
    plt.show()

    # Set q_iiwa_traj to start at t=0.
    shift_q_traj_to_start_at_minus_h(q_iiwa_traj, 0)
    q_mbp_traj = PiecewisePolynomial.FirstOrderHold(t_mbp, q_iiwa_log_mbp.T)

    e1, e_vec1, t_e1 = calc_error_integral(
        q_knots=q_iiwa_log_quasistatic,
        t=t_quasistatic,
        q_gt_traj=q_mbp_traj)
    print("Quasistatic vs MBP", e1)

    e2, e_vec2, t_e2 = calc_error_integral(
        q_knots=q_iiwa_log_mbp,
        t=t_mbp,
        q_gt_traj=q_iiwa_traj)
    print("MBP vs commanded", e2)

    e3, e_vec3, t_e3 = calc_error_integral(
        q_knots=q_iiwa_log_quasistatic,
        t=t_quasistatic,
        q_gt_traj=q_iiwa_traj)
    print("Quasistatic vs commanded", e3)

    e4, e_vec4, t_e4 = calc_error_integral(
        q_knots=q_iiwa_log_mbp,
        t=t_mbp,
        q_gt_traj=q_mbp_traj)
    print("MBP vs itself", e4)

    #%%
    plt.plot(t_e1, e_vec1, label="quasistatic")
    plt.plot(t_e2, e_vec2, label="mbp")
    plt.legend()
    plt.show()
