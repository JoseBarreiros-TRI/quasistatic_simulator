from sim_setup import *

#%% Run simulation.
loggers_dict_quasistatic_str, q_sys = run_quasistatic_sim(
    model_directive_path=model_directive_path,
    object_sdf_paths=object_sdf_dict,
    q_a_traj_dict_str=q_a_traj_dict_str,
    q0_dict_str=q0_dict_str,
    robot_stiffness_dict={robot_name: Kp},
    h=h,
    sim_params=quasistatic_sim_params,
    is_visualizing=True,
    real_time_rate=1.0)


#%% look into the plant.
plant = q_sys.plant
for model in q_sys.q_sim.models_all:
    print(model, plant.GetModelInstanceName(model),
          q_sys.q_sim.velocity_indices[model])


#%% construct q and v vectors of MBP from log.
name_to_model_dict = q_sys.q_sim.get_robot_name_to_model_instance_dict()
logger_qa = loggers_dict_quasistatic_str[robot_name]
logger_qu = loggers_dict_quasistatic_str[object_name]
q_log = np.zeros((T, plant.num_positions()))
v_log = np.zeros((T, plant.num_velocities()))
tau_a_log = np.zeros((T - 1, plant.num_actuated_dofs()))

for name, logger in loggers_dict_quasistatic_str.items():
    model = name_to_model_dict[name]
    for i, j in enumerate(q_sys.q_sim.velocity_indices[model]):
        q_log[:, j] = logger.data().T[:, i]

v_log[1:, :] = (q_log[1:, :] - q_log[:-1, :]) / h

for l in range(T - 1):
    qa_l = logger_qa.data().T[l]
    qa_l1_cmd = qa_traj.value((l + 1) * h).squeeze()
    tau_a_log[l] = Kp * (qa_l1_cmd - qa_l)

#%% plot q(t).
plt.figure()
plt.title('q(t)')
plt.plot(logger.sample_times(),  q_log[:, 0])
plt.plot(logger.sample_times(),  q_log[:, 1])
plt.grid(True)
plt.ylabel('t [s]')
plt.show()

#%% compare numerical and analytical gradients
q_sim = q_sys.q_sim
idx_u = name_to_model_dict[object_name]
idx_a = name_to_model_dict[robot_name]
q_u_logger = loggers_dict_quasistatic_str[object_name]
q_a_logger = loggers_dict_quasistatic_str[robot_name]

q_dict = {idx_a: np.array([0.8]),
          idx_u: np.array([1.0])}

q_a_cmd_dict = {idx_a: np.array([0.79])}

# numerical gradients.
dfdu_numerical = q_sim.calc_dfdu_numerical(
    q_dict=q_dict, qa_cmd_dict=q_a_cmd_dict, du=1e-3, h=h)

# kkt active gradients.
q_sim.update_mbp_positions(q_dict)
tau_ext_dict = q_sim.calc_tau_ext([])
q_sim.step(q_a_cmd_dict=q_a_cmd_dict,
           tau_ext_dict=tau_ext_dict,
           h=h, mode="qp_mp", requires_grad=True,
           grad_from_active_constraints=True)
dfdu_active = q_sim.get_Dq_nextDqa_cmd()

print("dfdu_numerical\n", dfdu_numerical)
print("dfdu_active\n", dfdu_active)
print("dfdx_active\n", q_sim.get_Dq_nextDq())

# kkt gradients
q_sim.update_mbp_positions(q_dict)
q_sim.step(q_a_cmd_dict=q_a_cmd_dict,
           tau_ext_dict=tau_ext_dict,
           h=h, mode="qp_mp", requires_grad=True,
           grad_from_active_constraints=False)

print("dfdu_kkt\n", q_sim.get_Dq_nextDqa_cmd())
print("dfdx_kkt\n", q_sim.get_Dq_nextDq())

# kkt active gradients.








