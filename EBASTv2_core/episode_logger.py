from pathlib import Path
import numpy as np
from EBASTv2_core.env import EBASTv2Env
import os
import shutil
from datetime import datetime


def _arr(x):
    return np.asarray(x, dtype=np.float32).reshape(-1)


def format_episode_recap(env: EBASTv2Env, episode_name="Episode Recap", time_decimals=3):
    lines = []

    n_transitions = len(env.action_list)

    lines.append("=" * 100)
    lines.append(episode_name)
    lines.append("=" * 100)
    lines.append(f"Own ship ID                              : {env.os_id}")
    lines.append(f"Target ship IDs                          : {env.ts_id}")
    lines.append(f"IW-enabled target ship IDs               : {env.ts_iw_id}")
    lines.append(f"Number of recorded observations          : {len(env.obs_list)}")
    lines.append(f"Number of recorded transitions           : {n_transitions}")
    lines.append("")
    
    relative_path               = env.srb_rel_path
    n_cases                     = env.spawn_requests_bank["n_cases"]
    n_training_cases            = env.spawn_requests_bank["start_eval_case_id"]
    n_evaluation_cases          = n_cases - n_training_cases
    north_bound                 = env.spawn_requests_bank["north_bound"]
    east_bound                  = env.spawn_requests_bank["east_bound"]
    rounded_north_bound         = env.spawn_requests_bank["rounded_north_bound"]
    rounded_east_bound          = env.spawn_requests_bank["rounded_east_bound"]
    
    case_idx                    = env.case_idx
    spawn_requests              = env.spawn_requests
    own_ship_initial            = env.own_ship_initial
    encounters                  = env.encounters
    
    lines.append("Spawn requests bank path:")
    lines.append(f"  -> .\{relative_path}.")
    lines.append(f"Split {n_cases} cases into {n_training_cases} training cases and {n_evaluation_cases} evaluation cases")
    lines.append(f"Use case {case_idx}:")
    lines.append("")
    lines.append(f"  ¤ {env.os_id} Initials:")
    lines.append("     - Start")
    lines.append(f"       > north                           : {spawn_requests[env.os_id]['north_route'][0]:.1f} [m]")
    lines.append(f"       > east                            : {spawn_requests[env.os_id]['east_route'][0]:.1f} [m]")
    lines.append("     - East")
    lines.append(f"       > north                           : {spawn_requests[env.os_id]['north_route'][-1]:.1f} [m]")
    lines.append(f"       > east                            : {spawn_requests[env.os_id]['east_route'][-1]:.1f} [m]")
    lines.append(f"     - sog                               : {own_ship_initial['sog']:.1f} [m/s]")
    lines.append(f"     - cog                               : {own_ship_initial['cog']:.1f} [deg]")
    lines.append(f"     - heading                           : {own_ship_initial['heading']:.1f} [deg]")
    lines.append(f"     - navStatus                         : {own_ship_initial['navStatus']}")
    lines.append("")
    
    for ts_id in list(encounters.keys()):
        lines.append(f"  ¤ {ts_id} encounter:")
        lines.append("     - Start")
        lines.append(f"       > north                           : {spawn_requests[ts_id]['north_route'][0]:.1f} [m]")
        lines.append(f"       > east                            : {spawn_requests[ts_id]['east_route'][0]:.1f} [m]")
        lines.append("     - East")
        lines.append(f"       > north                           : {spawn_requests[ts_id]['north_route'][-1]:.1f} [m]")
        lines.append(f"       > east                            : {spawn_requests[ts_id]['east_route'][-1]:.1f} [m]")
        lines.append(f"     - encounterType                     : {encounters[ts_id]['desiredEncounterType']}")
        lines.append(f"     - vectorTime                        : {encounters[ts_id]['vectorTime']:.1f} [minutes]")
        lines.append(f"     - beta                              : {encounters[ts_id]['beta']:.1f} [deg]")
        lines.append(f"     - relativeSpeed                     : {encounters[ts_id]['relativeSpeed']*own_ship_initial['sog']:.1f} [m/s]")
        lines.append("")
    
    lines.append("Positional observation space bound [min, max]:")
    lines.append(
        f" - NORTH BOUND: {[f'{x:.2f}' for x in north_bound]} "
        f"--rounded to-> {[f'{x:.2f}' for x in rounded_north_bound]}"
    )
    lines.append(
        f" - EAST  BOUND: {[f'{x:.2f}' for x in east_bound]} "
        f"--rounded to-> {[f'{x:.2f}' for x in rounded_east_bound]}"
    )
    lines.append("")

    for k in range(n_transitions):
        obs_before_norm = env.obs_list[k]
        obs_after_norm = env.obs_list[k + 1]

        obs_before = env._denormalize_observation(obs_before_norm)
        obs_after = env._denormalize_observation(obs_after_norm)

        action_norm = _arr(env.action_list[k])
        terminated = env.terminated_list[k]
        truncated = env.truncated_list[k]
        
        reward = env.reward_list[k]
        own_ship_collision_reward = env.reward_components["own_ship_collision_rewards"][k]
        own_ship_grounding_reward = env.reward_components["own_ship_grounding_rewards"][k]
        own_ship_navigational_failure_reward = env.reward_components["own_ship_navigational_failure_rewards"][k]
        own_ship_reaches_end_waypoint_reward = env.reward_components["own_ship_reaches_end_waypoint_rewards"][k]

        tar_ships_collision_reward = env.reward_components["tar_ships_collision_rewards"][k]
        tar_ships_grounding_reward = env.reward_components["tar_ships_grounding_rewards"][k]
        tar_ships_navigation_failure_reward = env.reward_components["tar_ships_navigation_failure_rewards"][k]
        tar_ships_reaches_end_waypoint_reward = env.reward_components["tar_ships_reaches_end_waypoint_rewards"][k]

        close_proximity_reward = env.reward_components["total_distance_rewards"][k]
        nearest_distance_reward = env.reward_components["nearest_distance_rewards"][k]
        nearest_distance_iw_reward = env.reward_components["nearest_distance_iw_rewards"][k]
        nearest_distance_roa_reward = env.reward_components["nearest_distance_roa_rewards"][k]

        scope_angle_request_done_reward = env.reward_components["scope_angle_request_done_rewards"][k]
        scope_angle_change_log_likelihood_reward = env.reward_components["scope_angle_change_log_likelihood_rewards"][k]
        
        intercept_scope_angle_reward = env.reward_components["intercept_scope_angle_rewards"][k]
        
        timestamp_before = round(env.event_timestamp_list[k] * 1e-9, time_decimals)
        timestamp_after = round(env.event_timestamp_list[k + 1] * 1e-9, time_decimals)

        action_phys = env._denormalize_action(action_norm)

        lines.append("-" * 100)
        lines.append(f"TRANSITION {k+1}")
        lines.append("-" * 100)
        lines.append(f"Simulation time before action            : {timestamp_before} s")
        lines.append(f"Simulation time after action             : {timestamp_after} s")
        lines.append(f"Terminated                               : {terminated}")
        lines.append(f"Truncated                                : {truncated}")
        lines.append(f"Total Reward                             : {reward}")
        lines.append("  > Termination Rewards")
        lines.append(f"    - Own Ship Collision                 : {own_ship_collision_reward}")
        lines.append(f"    - Own Ship Grounding                 : {own_ship_grounding_reward}")
        lines.append(f"    - Own Ship Navigational Failure      : {own_ship_navigational_failure_reward}")
        lines.append(f"    - Own Ship Reaches End Waypoint      : {own_ship_reaches_end_waypoint_reward}")
        lines.append(f"    - Target Ships Collision             : {tar_ships_collision_reward}")
        lines.append(f"    - Target Ships Grounding             : {tar_ships_grounding_reward}")
        lines.append(f"    - Target Ships Navigation Failure    : {tar_ships_navigation_failure_reward}")
        lines.append(f"    - Target Ships Reaches End Waypoint  : {tar_ships_reaches_end_waypoint_reward}")

        lines.append("  > Non-termination Rewards")
        lines.append(f"    - Scope Angle Request Done           : {scope_angle_request_done_reward:.4f}")
        lines.append(f"    - Scope Angle Change Log Likelihood  : {scope_angle_change_log_likelihood_reward}")
        lines.append(f"    - Intercepting Scope Angle           : {intercept_scope_angle_reward}")
        lines.append(f"    - Proximity based                    : {close_proximity_reward}")
        lines.append("      Average of three reward components")
        lines.append("      > Nearest Distance")
        lines.append(f"        {nearest_distance_reward}")
        lines.append("      > Nearest Distance IW")
        lines.append(f"        {nearest_distance_iw_reward}")
        lines.append("      > Nearest Distance ROA")
        lines.append(f"        {nearest_distance_roa_reward}")
        lines.append("")

        lines.append("[OBSERVATION BEFORE ACTION - DENORMALIZED]")
        _append_observation_recap(lines, env, obs_before)

        lines.append("[ACTION]")
        lines.append("A single action consists of [scope angle, scope length] * number of target ships with IW sampler enabled.")
        lines.append("")
        lines.append(f"Raw normalized action vector             : {action_norm.tolist()}")
        lines.append(f"Denormalized action vector               : {action_phys.tolist()}")
        lines.append("")

        for i, sid in enumerate(env.ts_iw_id):
            norm_scope_angle = action_norm[2 * i]
            norm_scope_length = action_norm[2 * i + 1]

            phys_scope_angle = action_phys[2 * i]
            phys_scope_length = action_phys[2 * i + 1]

            bounds = env.action_bound[sid]

            lines.append(f"IW action for target ship [{sid}]")
            lines.append(f"  Normalized scope angle                 : {float(norm_scope_angle)}")
            lines.append(f"  Normalized scope length                : {float(norm_scope_length)}")
            lines.append(f"  Scope angle deg                        : {float(phys_scope_angle)}")
            lines.append(f"  Scope length m                         : {float(phys_scope_length)}")
            lines.append(f"  Scope angle bounds deg                 : [{bounds['min'][0]}, {bounds['max'][0]}]")
            lines.append(f"  Scope length bounds m                  : [{bounds['min'][1]}, {bounds['max'][1]}]")
            lines.append("")

        lines.append("[OBSERVATION AFTER ACTION - DENORMALIZED]")
        _append_observation_recap(lines, env, obs_after)

    return "\n".join(lines)


def _append_observation_recap(lines, env, obs):
    lines.append("")

    lines.append(
        f"COLAV states                             : "
        f"[{', '.join(str(x) for x in _arr(obs['colav_states']))}] [%, deg]"
    )
    lines.append(
        f"Action masks                             : "
        f"[{', '.join(str(int(x)) for x in _arr(obs['action_masks']))}]"
    )
    lines.append("")

    own_pos = _arr(obs["own_ship_pos"])
    lines.append(f"Own ship [{env.os_id}]")
    lines.append(
        f"  Position/state vector                  : "
        f"[{', '.join(f'{x:.1f}' for x in own_pos)}] [m, m, deg]"
    )
    lines.append(
        f"  Cross-track error                      : "
        f"{', '.join(f'{x:.1f} [m]' for x in _arr(obs['own_ship_e_ct']))}"
    )
    lines.append(
        f"  Rudder angle                           : "
        f"{', '.join(f'{x:.1f} [deg]' for x in _arr(obs['own_ship_rud_ang']))}"
    )
    lines.append(
        f"  Throttle                               : "
        f"{', '.join(f'{x:-1f}' for x in _arr(obs['own_ship_throttle']))} [%]"
    )
    lines.append(
        f"  Forward speed                          : "
        f"{', '.join(f'{x:-1f} [m/s]' for x in _arr(obs['own_ship_forward_speed']))}"
    )
    lines.append("")

    # Conver relative positions to own ship into real positions
    rel_tar_pos_flat    = _arr(obs["rel_tar_ships_pos"])
    rel_tar_pos         = rel_tar_pos_flat.reshape(-1,3)    # reshape into n row x 3 column again
    tar_pos             = rel_tar_pos.copy()
    tar_pos[:, :2]     += own_pos[:2]                       # convert rel_north and rel_east into north and east
    tar_pos_flat        = tar_pos.reshape(-1)               # flatten the altered  array again
    
    tar_speed           = _arr(obs["tar_ships_forward_speed"])

    lines.append("Target ships")
    for i, sid in enumerate(env.ts_id):
        pos_i   = tar_pos_flat[3 * i: 3 * i + 3]
        speed_i = tar_speed[i]

        lines.append(f"  Target ship [{sid}]")
        lines.append(
            f"    Position/state vector                : "
            f"[{', '.join(f'{x:.1f}' for x in pos_i)}] [m, m, deg]"
        )
        lines.append(f"    Forward speed                        : {float(speed_i):-1f} [m/s]")

        if sid in env.ts_iw_id:
            iw_idx = env.ts_iw_id.index(sid)
            lines.append(f"    IW-enabled                           : True")
            lines.append(f"    Action mask                          : {int(_arr(obs['action_masks'])[iw_idx])}")
            lines.append(f"    Remaining requests                   : {int(_arr(obs['remaining_requests'])[iw_idx])}")
        else:
            lines.append(f"    IW-enabled                           : False")

    lines.append("")


def log_episode_recap(env, log_path, time_decimals=3):
    log_path = Path(log_path)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    text = format_episode_recap(
        env,
        time_decimals=time_decimals,
    )

    log_path.write_text(text, encoding="utf-8")

    return log_path


def log_training_args(ROOT, args, log_path, save_reward_function=False):
    exp_dir = os.path.dirname(log_path)
    os.makedirs(exp_dir, exist_ok=True)

    # Save args
    with open(log_path, "w") as f:
        f.write(f"Training Run: {datetime.now()}\n")
        f.write("=" * 80 + "\n")

        for key, value in sorted(vars(args).items()):
            f.write(f"{key}: {value}\n")

        f.write("=" * 80 + "\n")

    # Save a copy of the reward function
    if save_reward_function:
        reward_function_path = ROOT / "EBASTv2_core" / "reward_function.py"
        reward_copy_path = os.path.join(
            exp_dir,
            os.path.basename(reward_function_path)
        )
        shutil.copy2(reward_function_path, reward_copy_path)