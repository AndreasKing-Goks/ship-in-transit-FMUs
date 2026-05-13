from pathlib import Path
import numpy as np


def _arr(x):
    return np.asarray(x, dtype=np.float32).reshape(-1)


def format_episode_recap(env, episode_name="Episode Recap", time_decimals=3):
    lines = []

    n_transitions = len(env.action_list)

    lines.append("=" * 100)
    lines.append(episode_name)
    lines.append("=" * 100)
    lines.append(f"Own ship ID: {env.os_id}")
    lines.append(f"Target ship IDs: {env.ts_id}")
    lines.append(f"IW-enabled target ship IDs: {env.ts_iw_id}")
    lines.append(f"Number of recorded observations: {len(env.obs_list)}")
    lines.append(f"Number of recorded transitions: {n_transitions}")
    lines.append("")

    for k in range(n_transitions):
        obs_before_norm = env.obs_list[k]
        obs_after_norm = env.obs_list[k + 1]

        obs_before = env._denormalize_observation(obs_before_norm)
        obs_after = env._denormalize_observation(obs_after_norm)

        action_norm = _arr(env.action_list[k])
        reward = env.reward_list[k]
        terminated = env.terminated_list[k]
        truncated = env.truncated_list[k]

        timestamp_before = round(env.event_timestamp_list[k] * 1e-9, time_decimals)
        timestamp_after = round(env.event_timestamp_list[k + 1] * 1e-9, time_decimals)

        action_phys = env._denormalize_action(action_norm)

        lines.append("-" * 100)
        lines.append(f"TRANSITION {k}")
        lines.append("-" * 100)
        lines.append(f"Simulation time before action: {timestamp_before} s")
        lines.append(f"Simulation time after action: {timestamp_after} s")
        lines.append(f"Reward: {reward}")
        lines.append(f"Terminated: {terminated}")
        lines.append(f"Truncated: {truncated}")
        lines.append("")

        lines.append("[OBSERVATION BEFORE ACTION - DENORMALIZED]")
        _append_observation_recap(lines, env, obs_before)

        lines.append("[ACTION]")
        lines.append("")
        lines.append(f"Raw normalized action vector: {action_norm.tolist()}")
        lines.append(f"Denormalized action vector: {action_phys.tolist()}")
        lines.append("")

        for i, sid in enumerate(env.ts_iw_id):
            norm_scope_angle = action_norm[2 * i]
            norm_scope_length = action_norm[2 * i + 1]

            phys_scope_angle = action_phys[2 * i]
            phys_scope_length = action_phys[2 * i + 1]

            bounds = env.action_bound[sid]

            lines.append(f"IW action for target ship [{sid}]")
            lines.append(f"  Normalized scope angle: {float(norm_scope_angle)}")
            lines.append(f"  Normalized scope length: {float(norm_scope_length)}")
            lines.append(f"  Scope angle deg: {float(phys_scope_angle)}")
            lines.append(f"  Scope length m: {float(phys_scope_length)}")
            lines.append(f"  Scope angle bounds deg: [{bounds['min'][0]}, {bounds['max'][0]}]")
            lines.append(f"  Scope length bounds m: [{bounds['min'][1]}, {bounds['max'][1]}]")
            lines.append("")

        lines.append("[OBSERVATION AFTER ACTION - DENORMALIZED]")
        _append_observation_recap(lines, env, obs_after)

    return "\n".join(lines)


def _append_observation_recap(lines, env, obs):
    lines.append("")

    lines.append(f"COLAV states: {_arr(obs['colav_states']).tolist()}")
    lines.append(f"Action masks: {_arr(obs['action_masks']).astype(int).tolist()}")
    lines.append(f"Remaining requests: {_arr(obs['remaining_requests']).tolist()}")
    lines.append("")

    own_pos = _arr(obs["own_ship_pos"])
    lines.append(f"Own ship [{env.os_id}]")
    lines.append(f"  Position/state vector: {own_pos.tolist()}")
    lines.append(f"  Cross-track error: {_arr(obs['own_ship_e_ct']).tolist()}")
    lines.append(f"  Rudder angle: {_arr(obs['own_ship_rud_ang']).tolist()}")
    lines.append(f"  Throttle: {_arr(obs['own_ship_throttle']).tolist()}")
    lines.append(f"  Forward speed: {_arr(obs['own_ship_forward_speed']).tolist()}")
    lines.append("")

    tar_pos_flat = _arr(obs["tar_ships_pos"])
    tar_speed = _arr(obs["tar_ships_forward_speed"])

    lines.append("Target ships")
    for i, sid in enumerate(env.ts_id):
        pos_i = tar_pos_flat[3 * i: 3 * i + 3]
        speed_i = tar_speed[i]

        lines.append(f"  Target ship [{sid}]")
        lines.append(f"    Position/state vector: {pos_i.tolist()}")
        lines.append(f"    Forward speed: {float(speed_i)}")

        if sid in env.ts_iw_id:
            iw_idx = env.ts_iw_id.index(sid)
            lines.append(f"    IW-enabled: True")
            lines.append(f"    Action mask: {int(_arr(obs['action_masks'])[iw_idx])}")
            lines.append(f"    Remaining requests: {float(_arr(obs['remaining_requests'])[iw_idx])}")
        else:
            lines.append(f"    IW-enabled: False")

    lines.append("")


def log_episode_recap(env, log_path, time_decimals=3):
    log_path = Path(log_path)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    text = format_episode_recap(
        env,
        time_decimals=time_decimals
    )

    log_path.write_text(text, encoding="utf-8")

    return log_path