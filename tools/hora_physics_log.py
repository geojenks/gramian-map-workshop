# Physics logger for the Hora policy: W16 S1's telemetry script from Wave_manip
# (experiment_log/scripts/w16s1_hora_telemetry_pos.py) with a read-only hook on
# env.update_low_level_control, which saves per physics step the object state,
# joint angles, velocities and applied torques, link poses, Jacobians, mass matrix
# and every contact. The policy, checkpoint and seed are unchanged.
#
# Run in WSL Ubuntu, conda env "hora", cwd ~/hora, with
# LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH:
#     python tools/hora_physics_log.py --seed 42 --steps 600 --out data/rl_logs/hora_seed42_30s.npz
# It writes <out> (the ordinary telemetry) and <out>_physics.npz.
"""W16 S1 — Hora contact telemetry, POSITION-ENRICHED (superset of w13s1).

Identical rollout to experiment_log/scripts/w13s1_hora_telemetry.py (same
checkpoint, same overrides, same seed discipline) — the banked Phase C schema
fields are written UNCHANGED so w13s1_cluster_contrast_v3.load_events consumes
this npz as before.  Added fields, all measured from the same contacts/steps:

    contact_pos_world (N,3)   world-frame contact point
    contact_pos_obj   (N,3)   contact point in the OBJECT frame (localPos)
    contact_nrm_world (N,3)   contact normal (world), if the dtype exposes it
    tip_pos           (T,4,3) world positions of the four tip links
                              (link_3/7/11/15), per step
    palm_pos/palm_quat(T,3)/(T,4)  palm (base) body pose, per step
    hand_body_names   (B,)    all hand body names (mount reference)
    hand0_pos/quat    (B,3)/(B,4)  every hand body pose at segment start
    obj_scale         scalar  from priv_info_buf if reachable, else NaN

Why: the A3 LEAP campaign (w15s8..s25) leaned on per-contact 3D positions
(link census, contact heights/radii, underside topology); the w13s1 Hora npz
carries azimuth + link name only.  This closes that gap for the Hora->Allegro
same-hand reconstruction (Paper 1, A3 second half, George's go 14 Sep 2026).

Consistency check vs the banked w13s1 npz (same seed => same rollout): the
contact count and azimuth series of the overlapping fields are compared by the
scout analysis, not here (this script never reads the banked npz).

MUST run inside WSL2 Ubuntu, conda env "hora", cwd = ~/hora, with
LD_LIBRARY_PATH including $CONDA_PREFIX/lib (see run_hora_telemetry_pos.sh).
"""
import argparse
import os
import sys

import isaacgym  # noqa: F401  (must precede torch import)
import numpy as np
import torch

HORA_DIR = os.path.expanduser("~/hora")
sys.path.insert(0, HORA_DIR)

TIP_LINKS = ["link_3", "link_7", "link_11", "link_15"]


def quat_apply_np(q, v):
    """Rotate vector v by quaternion q (IsaacGym xyzw order). numpy, single."""
    x, y, z, w = q
    qv = np.array([x, y, z])
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--steps", type=int, default=2400, help="policy steps (20 Hz)")
    ap.add_argument("--object", default="cylinder_default")
    ap.add_argument("--checkpoint",
                    default="outputs/AllegroHandHora/hora_v0.0.2/hora_v0.0.2/stage2_nn/model_last.ckpt")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    os.chdir(HORA_DIR)

    from hydra import compose, initialize_config_dir
    from omegaconf import OmegaConf

    OmegaConf.register_new_resolver("eq", lambda x, y: x.lower() == y.lower())
    OmegaConf.register_new_resolver("contains", lambda x, y: x.lower() in y.lower())
    OmegaConf.register_new_resolver("if", lambda pred, a, b: a if pred else b)
    OmegaConf.register_new_resolver(
        "resolve_default", lambda default, arg: default if arg == "" else arg)

    overrides = [
        "task=AllegroHandHora",
        "headless=True",
        "test=True",
        "pipeline=cpu",
        "sim_device=cpu",
        "num_envs=1",
        f"seed={args.seed}",
        f"task.env.object.type={args.object}",
        "task.on_evaluation=False",
        f"task.env.episodeLength={args.steps + 10}",
        "task.env.randomization.randomizeScaleList=[0.8,0.82,0.84,0.86,0.88,0.9]",
        "task.env.randomization.jointNoiseScale=0.005",
        "task.env.reset_height_threshold=0.6",
        "task.env.forceScale=0",
        "train.algo=ProprioAdapt",
        "train.ppo.priv_info=True",
        "train.ppo.proprio_adapt=True",
        "train.ppo.output_name=AllegroHandHora/w16s1_telemetry",
        f"checkpoint={os.path.abspath(args.checkpoint)}",
    ]
    with initialize_config_dir(config_dir=os.path.join(HORA_DIR, "configs"),
                               job_name="w16s1_hora_telemetry"):
        config = compose(config_name="config", overrides=overrides)

    from hora.algo.padapt.padapt import ProprioAdapt
    from hora.tasks import isaacgym_task_map
    from hora.utils.misc import set_np_formatting, set_seed
    from hora.utils.reformat import omegaconf_to_dict

    set_np_formatting()
    set_seed(args.seed)

    env = isaacgym_task_map[config.task_name](
        config=omegaconf_to_dict(config.task),
        sim_device=config.sim_device,
        graphics_device_id=config.graphics_device_id,
        headless=config.headless,
    )
    output_dir = os.path.join("outputs", config.train.ppo.output_name)
    os.makedirs(output_dir, exist_ok=True)
    agent = ProprioAdapt(env, output_dir, full_config=config)
    agent.restore_test(config.train.load_path)
    agent.set_eval()

    env_ptr = env.envs[0]
    obj_body_idx = int(env.object_rb_handles[0].item())
    hand_body_names = env.gym.get_actor_rigid_body_names(env_ptr, 0)
    step_dt = env.control_freq_inv * env.dt  # 6 * 1/120 = 0.05 s
    print(f"[telemetry] object env-body idx {obj_body_idx}; "
          f"{len(hand_body_names)} hand bodies; step_dt {step_dt:.4f}s")

    # --- W16 additions: body indices for tips + palm -----------------------
    # IsaacGym suffixes body names with the env instance (banked contact_link
    # values are 'link_3.0' etc.); normalize before matching.
    base_names = [n.split(".")[0] for n in hand_body_names]
    tip_idx = [base_names.index(n) for n in TIP_LINKS]
    palm_name = next((n for n in base_names if "palm" in n.lower()
                      or "base" in n.lower() or n == "link_0"),
                     base_names[0])
    palm_idx = base_names.index(palm_name)
    print(f"[telemetry] tips {list(zip(TIP_LINKS, tip_idx))}; "
          f"palm '{palm_name}' idx {palm_idx}")

    # object scale from the privileged buffer, if the mapping is reachable
    obj_scale = float("nan")
    try:
        sidx = env.priv_info_dict["obj_scale"]
        obj_scale = float(np.atleast_1d(
            env.priv_info_buf[0, sidx].cpu().numpy())[0])
    except Exception as e:  # noqa: BLE001 - recorded honestly as NaN
        print(f"[telemetry] obj_scale not reachable ({e!r}); NaN")
    print(f"[telemetry] obj_scale {obj_scale}")

    from isaacgym import gymapi  # here: isaacgym must be imported first

    t_list, pos_list, quat_list, angvel_list, done_list = [], [], [], [], []
    ct_t, ct_theta, ct_link = [], [], []
    ct_pw, ct_po, ct_nw = [], [], []
    tip_list, palm_p_list, palm_q_list = [], [], []
    hp_list, hq_list = [], []
    contact_fields_printed = False

    # --- every-physics-step recorder (read-only) ---------------------------
    from isaacgym import gymtorch
    actor_name = env.gym.get_actor_name(env_ptr, 0)
    jac_t = gymtorch.wrap_tensor(env.gym.acquire_jacobian_tensor(env.sim, actor_name))
    mm_t = gymtorch.wrap_tensor(env.gym.acquire_mass_matrix_tensor(env.sim, actor_name))
    obj_handle = env.gym.find_actor_handle(env_ptr, "object")
    rbp = env.gym.get_actor_rigid_body_properties(env_ptr, obj_handle)[0]
    obj_mass = float(rbp.mass)
    obj_com = [rbp.com.x, rbp.com.y, rbp.com.z]
    obj_I = [[rbp.inertia.x.x, rbp.inertia.x.y, rbp.inertia.x.z],
             [rbp.inertia.y.x, rbp.inertia.y.y, rbp.inertia.y.z],
             [rbp.inertia.z.x, rbp.inertia.z.y, rbp.inertia.z.z]]
    obj_fric = [float(p.friction) for p in env.gym.get_actor_rigid_shape_properties(env_ptr, obj_handle)]
    hand_fric = [float(p.friction) for p in env.gym.get_actor_rigid_shape_properties(env_ptr, 0)]
    dof_names = env.gym.get_actor_dof_names(env_ptr, 0)
    print(f"[physics] mass {obj_mass} com {obj_com} I {obj_I} fric {obj_fric} handfric {hand_fric[:3]} jac {tuple(jac_t.shape)} mm {tuple(mm_t.shape)} dofs {dof_names}")
    P = dict(tick=[], obj=[], q=[], qd=[], tau=[], body=[], jac=[], mm=[], step=[])
    Cts = []
    orig_llc = env.update_low_level_control
    cur_step = [-1]
    def llc_hook():
        tau_prev = env.torques[0].cpu().numpy().copy()
        orig_llc()
        k = len(P["tick"])
        env.gym.refresh_jacobian_tensors(env.sim)
        env.gym.refresh_mass_matrix_tensors(env.sim)
        P["tick"].append(k); P["step"].append(cur_step[0])
        P["obj"].append(env.root_state_tensor[env.object_indices[0]].cpu().numpy().copy())
        P["q"].append(env.allegro_hand_dof_pos[0].cpu().numpy().copy())
        P["qd"].append(env.allegro_hand_dof_vel[0].cpu().numpy().copy())
        P["tau"].append(tau_prev)
        rb = env.gym.get_actor_rigid_body_states(env_ptr, 0, gymapi.STATE_ALL)
        P["body"].append(np.array([[*[rb[j]["pose"]["p"][a] for a in range(3)], *[rb[j]["pose"]["r"][a] for a in range(4)]] for j in range(len(rb))], np.float32))
        P["jac"].append(jac_t[0].cpu().numpy().astype(np.float32).copy())
        P["mm"].append(mm_t[0].cpu().numpy().astype(np.float32).copy())
        for c in env.gym.get_env_rigid_contacts(env_ptr):
            b0, b1 = int(c["body0"]), int(c["body1"])
            if obj_body_idx not in (b0, b1):
                continue
            Cts.append([k, b0, b1, float(c["lambda"]), c["normal"]["x"], c["normal"]["y"], c["normal"]["z"],
                        c["localPos0"]["x"], c["localPos0"]["y"], c["localPos0"]["z"],
                        c["localPos1"]["x"], c["localPos1"]["y"], c["localPos1"]["z"],
                        c["lambdaFriction"][0], c["lambdaFriction"][1], float(c["friction"])])
    env.update_low_level_control = llc_hook

    obs_dict = env.reset()
    for i in range(args.steps):
        with torch.no_grad():
            input_dict = {
                "obs": agent.running_mean_std(obs_dict["obs"]),
                "proprio_hist": agent.sa_mean_std(obs_dict["proprio_hist"].detach()),
            }
            mu = agent.model.act_inference(input_dict)
            mu = torch.clamp(mu, -1.0, 1.0)
        cur_step[0] = i
        obs_dict, r, done, info = env.step(mu)

        t_now = (i + 1) * step_dt
        obj_pos = env.object_pos[0].cpu().numpy().astype(np.float64)
        obj_quat = env.object_rot[0].cpu().numpy().astype(np.float64)
        obj_angvel = env.object_angvel[0].cpu().numpy().astype(np.float64)
        t_list.append(t_now)
        pos_list.append(obj_pos)
        quat_list.append(obj_quat)
        angvel_list.append(obj_angvel)
        done_list.append(bool(done[0].item()))

        # --- W16 addition: per-step tip + palm poses -----------------------
        rb = env.gym.get_actor_rigid_body_states(env_ptr, 0, gymapi.STATE_ALL)
        tips = [[float(rb[j]["pose"]["p"][a]) for a in range(3)] for j in tip_idx]
        tip_list.append(tips)
        palm_p_list.append([float(rb[palm_idx]["pose"]["p"][a]) for a in range(3)])
        palm_q_list.append([float(rb[palm_idx]["pose"]["r"][a]) for a in range(4)])
        # every hand body pose every step: the segment start (the fit datum
        # for the MuJoCo mount) is only known after the run, so record all
        hp_list.append([[float(rb[j]["pose"]["p"][a]) for a in range(3)]
                        for j in range(len(hand_body_names))])
        hq_list.append([[float(rb[j]["pose"]["r"][a]) for a in range(4)]
                        for j in range(len(hand_body_names))])

        contacts = env.gym.get_env_rigid_contacts(env_ptr)
        if len(contacts) and not contact_fields_printed:
            print(f"[telemetry] contact dtype fields: {contacts.dtype.names}")
            contact_fields_printed = True
        for c in contacts:
            b0, b1 = int(c["body0"]), int(c["body1"])
            if b0 == obj_body_idx and 0 <= b1 < obj_body_idx:
                local = np.array([c["localPos0"]["x"], c["localPos0"]["y"],
                                  c["localPos0"]["z"]])
                other = b1
            elif b1 == obj_body_idx and 0 <= b0 < obj_body_idx:
                local = np.array([c["localPos1"]["x"], c["localPos1"]["y"],
                                  c["localPos1"]["z"]])
                other = b0
            else:
                continue
            world = obj_pos + quat_apply_np(obj_quat, local)
            theta = float(np.arctan2(world[1] - obj_pos[1], world[0] - obj_pos[0]))
            ct_t.append(t_now)
            ct_theta.append(theta)
            ct_link.append(hand_body_names[other])
            # --- W16 additions: 3D position + normal -----------------------
            ct_pw.append(world)
            ct_po.append(local)
            if "normal" in contacts.dtype.names:
                ct_nw.append([float(c["normal"]["x"]), float(c["normal"]["y"]),
                              float(c["normal"]["z"])])
            else:
                ct_nw.append([np.nan, np.nan, np.nan])

        if (i + 1) % 200 == 0:
            wz = np.degrees(np.mean([a[2] for a in angvel_list[-200:]]))
            print(f"[telemetry] step {i + 1}/{args.steps}  contacts so far "
                  f"{len(ct_t)}  mean omega_z(last 10s) {wz:+.2f} dps  "
                  f"resets {sum(done_list)}")

    # longest reset-free segment
    done_arr = np.array(done_list, bool)
    t_arr = np.array(t_list)
    boundaries = [0] + [k + 1 for k in np.nonzero(done_arr)[0]] + [len(t_arr)]
    seg = max(zip(boundaries[:-1], boundaries[1:]), key=lambda ab: ab[1] - ab[0])
    s0, s1 = seg
    t_lo, t_hi = t_arr[s0], t_arr[s1 - 1]
    print(f"[telemetry] longest reset-free segment: steps {s0}..{s1 - 1} "
          f"({t_hi - t_lo:.1f}s of {t_arr[-1]:.1f}s)")

    ct_t = np.array(ct_t)
    ct_m = (ct_t >= t_lo) & (ct_t <= t_hi)
    out = {
        # --- banked Phase C schema, unchanged names ------------------------
        "t": t_arr[s0:s1] - t_lo,
        "cube_pos": np.array(pos_list)[s0:s1],
        "cube_quat": np.array(quat_list)[s0:s1],
        "cube_angvel": np.array(angvel_list)[s0:s1],
        "step_dt": np.float64(step_dt),
        "contact_t": ct_t[ct_m] - t_lo,
        "contact_theta": np.array(ct_theta)[ct_m],
        "contact_link": np.array(ct_link)[ct_m],
        "env_seed": np.int64(args.seed),
        "object_type": np.str_(args.object),
        "checkpoint": np.str_(args.checkpoint),
        "n_resets": np.int64(int(done_arr.sum())),
        "source": np.str_("hora_isaacgym_cpu_pipeline"),
        # --- W16 additions -------------------------------------------------
        "contact_pos_world": np.array(ct_pw)[ct_m],
        "contact_pos_obj": np.array(ct_po)[ct_m],
        "contact_nrm_world": np.array(ct_nw)[ct_m],
        "tip_pos": np.array(tip_list)[s0:s1],
        "palm_pos": np.array(palm_p_list)[s0:s1],
        "palm_quat": np.array(palm_q_list)[s0:s1],
        "hand_body_names": np.array(hand_body_names),
        "hand_pos": np.array(hp_list)[s0:s1],
        "hand_quat": np.array(hq_list)[s0:s1],
        "obj_scale": np.float64(obj_scale),
        "schema": np.str_("phaseC_learned+w16pos"),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    np.savez_compressed(args.out, **out)
    np.savez_compressed(args.out.replace(".npz", "_physics.npz"), dt=env.dt, obj_mass=obj_mass, obj_com=np.array(obj_com), obj_I=np.array(obj_I), obj_fric=np.array(obj_fric), hand_fric=np.array(hand_fric), dof_names=np.array(dof_names), body_names=np.array(hand_body_names), obj_body=obj_body_idx, done=done_arr, contacts=np.array(Cts, np.float64), **{k: np.array(v) for k, v in P.items()})
    n_ct = int(ct_m.sum())
    wz_seg = float(np.degrees(np.mean(out["cube_angvel"][:, 2])))
    print(f"[telemetry] wrote {args.out}: {n_ct} contacts over "
          f"{t_hi - t_lo:.1f}s, segment mean omega_z {wz_seg:+.2f} dps")
    print("TELEMETRY_DONE")


if __name__ == "__main__":
    main()
