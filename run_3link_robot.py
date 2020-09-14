import time

from quasistatic_simulator import *
from meshcat_camera_utils import SetOrthographicCameraYZ

#%%
q_sim = QuasistaticSimulator(CreatePlantFor2dArmWith2dBox, nd_per_contact=2)
SetOrthographicCameraYZ(q_sim.viz.vis)


#%%
q_a = np.array([np.pi / 2, -np.pi / 2, -np.pi / 2])
q_u = np.array([1.7, 0.5, 0])
q = np.hstack([q_u, q_a])
q_sim.UpdateConfiguration(q)
q_sim.DrawCurrentConfiguration()

#%%
h = 0.001
tau_u_ext = np.array([0., -10, 0])
n_steps = 500

input("start?")
for i in range(n_steps):
    # dr = np.min([0.001 * i, 0.02])
    # q_a_cmd = np.array([-r * 1.1 + dr, r * 1.1 - dr, -0.002 * i])
    q_a_cmd = np.array([np.pi / 2, -np.pi / 2, -np.pi / 2]) + h * i
    dq_a, dq_u, beta, constraint_values, result = q_sim.StepAnitescu(
        q, q_a_cmd, tau_u_ext, h)

    # Update q
    q += np.hstack([dq_u, dq_a])
    q_sim.UpdateConfiguration(q)
    q_sim.DrawCurrentConfiguration()

    # logging
    time.sleep(h * 10)
    # input("next?")

#%%
n_c, n_d, n_f, Jn_u_q, Jn_u_v, Jn_a, Jf_u_q, Jf_u_v, Jf_a, phi = \
    q_sim.CalcContactJacobians(0.01)
query_object = q_sim.scene_graph.get_query_output_port().Eval(q_sim.context_sg)
signed_distance_pairs = \
    query_object.ComputeSignedDistancePairwiseClosestPoints()

plant = q_sim.plant
inspector = query_object.inspector()
for i, sdp in enumerate(signed_distance_pairs):
    print("contact%d"%i)
    print(inspector.GetNameByGeometryId(sdp.id_A))
    print(inspector.GetNameByGeometryId(sdp.id_B))
    print(inspector.GetFrameId(sdp.id_A))
    print(inspector.GetFrameId(sdp.id_B))
    print("distance: ", sdp.distance)
    print("p_AC_a: ", sdp.p_ACa)
    print("p_BC_b: ", sdp.p_BCb)
    print("nhat_BA_W", sdp.nhat_BA_W)
    print("")


