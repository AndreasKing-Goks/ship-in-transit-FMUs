from pathlib import Path
import numpy as np


def evaluate_failure_cases(
    env,
    model,
    recap_path,
    case_indices=range(100),
    deterministic=True,
):
    """
    Evaluate a trained RecurrentPPO model over specified encounter cases.

    For each episode, classify the result as:
        - no_collision
        - target_collision
        - collision
        - nav_failure
        - other

    A recap containing the result of every episode and the final
    percentages is written to ``recap_path``.

    Parameters
    ----------
    env :
        EBASTv2 environment.

    model :
        Trained RecurrentPPO model.

    recap_path : str or Path
        Path where the evaluation recap will be stored.

    case_indices : iterable, optional
        Encounter case indices to evaluate.

    deterministic : bool, optional
        Whether to use deterministic model prediction.

    Returns
    -------
    dict
        Dictionary containing:
            - status_count
            - percentages
            - episode_results
    """

    recap_path = Path(recap_path)
    recap_path.parent.mkdir(parents=True, exist_ok=True)

    # Convert to list so that it can safely be reused
    case_indices = list(case_indices)

    status_count = {
        "no_collision": 0,
        "target_collision": 0,
        "collision": 0,
        "nav_failure": 0,
        "other": 0,
    }

    episode_results = []
    # ==========================================================
    # Run evaluation
    # ==========================================================
    for episode_number, idx in enumerate(case_indices, start=1):
        print(
            f"Evaluating episode {episode_number}/{len(case_indices)} "
            f"(case {idx})"
        )
        obs, _ = env.reset(
            seed=None,
            specific_case_idx=idx,
        )
        # Reset recurrent state at the start of every episode
        lstm_states = None

        # One environment -> one episode-start flag
        episode_starts = np.ones((1,), dtype=bool)
        step_count = 0

        while True:
            action, lstm_states = model.predict(
                obs,
                state=lstm_states,
                episode_start=episode_starts,
                deterministic=deterministic,
            )
            obs, reward, terminated, truncated, info = env.step(action)
            step_count += 1
            done = bool(terminated) or bool(truncated)
            episode_starts = np.array(
                [done],
                dtype=bool,
            )
            if done:
                break

        # ======================================================
        # Determine episode outcome
        # ======================================================
        collision_os = bool(
            env.instance.termination_flags[
                "collision_flags"
            ]["OS0"]
        )
        nav_failure_os = bool(
            env.instance.termination_flags[
                "nav_fail_flags"
            ]["OS0"]
        )
        any_collision = bool(
            env.instance.any_ship_collides
        )
        reached_end = bool(
            env.instance.own_ship_reaches_end_waypoint
            or env.instance.all_ship_reaches_end_waypoint
        )

        # ======================================================
        # Classify episode
        # ======================================================
        if collision_os:
            status = "collision"

        elif nav_failure_os:
            status = "nav_failure"

        elif any_collision and not collision_os:
            status = "target_collision"

        elif reached_end:
            status = "no_collision"

        else:
            status = "other"

        status_count[status] += 1

        # Store complete episode recap
        episode_result = {
            "episode": episode_number,
            "case_idx": idx,
            "status": status,
            "steps": step_count,
            "terminated": bool(terminated),
            "truncated": bool(truncated),
            "collision_os": collision_os,
            "nav_failure_os": nav_failure_os,
            "any_collision": any_collision,
            "reached_end": reached_end,
        }

        episode_results.append(episode_result)
        print(f"    Result: {status}")

        if status == "other":
            print(f"    WARNING: case {idx} could not be classified.")

    # ==========================================================
    # Calculate statistics
    # ==========================================================
    total = sum(status_count.values())
    percentages = {}
    for status, count in status_count.items():
        if total > 0:
            percentage = count / total * 100.0
        else:
            percentage = 0.0
        percentages[status] = percentage

    # ==========================================================
    # Write recap
    # ==========================================================
    with recap_path.open("w", encoding="utf-8") as file:
        file.write("========================================\n")
        file.write("EBASTv2 FAILURE EVALUATION RECAP\n")
        file.write("========================================\n\n")

        file.write(f"Number of evaluated episodes: {total}\n\n")

        # ------------------------------------------------------
        # Individual episodes
        # ------------------------------------------------------
        file.write("EPISODE RESULTS\n")
        file.write("----------------------------------------\n")

        for result in episode_results:
            file.write(f"Episode      : {result['episode']}\n")
            file.write(f"Case index   : {result['case_idx']}\n")
            file.write(f"Status       : {result['status']}\n")
            file.write(f"Steps        : {result['steps']}\n")
            file.write(f"Terminated   : {result['terminated']}\n")
            file.write(f"Truncated    : {result['truncated']}\n")
            file.write(f"OS collision : {result['collision_os']}\n")
            file.write(f"OS nav fail  : {result['nav_failure_os']}\n")
            file.write(f"Any collision: {result['any_collision']}\n")
            file.write(f"Reached end  : {result['reached_end']}\n")
            file.write("----------------------------------------\n")

        # ------------------------------------------------------
        # Summary
        # ------------------------------------------------------
        file.write("\nSUMMARY\n")
        file.write("========================================\n")

        for status, count in status_count.items():
            percentage = percentages[status]
            file.write(
                f"{status:<20}: "
                f"{count:4d}/{total:4d} "
                f"({percentage:7.2f} %)\n"
            )

    # ==========================================================
    # Print summary
    # ==========================================================
    print("\n========================================")
    print("Evaluation Results")
    print("========================================")

    for status, count in status_count.items():
        print(
            f"{status:<20}: "
            f"{count:4d}/{total:4d} "
            f"({percentages[status]:7.2f} %)"
        )

    print(f"\nEvaluation recap saved to:\n{recap_path}")

    # ==========================================================
    # Return results
    # ==========================================================
    return {
        "status_count": status_count,
        "percentages": percentages,
        "episode_results": episode_results,
    }
    

# # Reset the trained model
# case_idx        = range(100)

# status_count    ={
#     "no_collision": 0,
#     "target_collision": 0,
#     "collision": 0,
#     "nav_failure": 0,
# }

# for idx in case_idx:
#     print(idx)
#     seed        = None
#     obs, _      = env.reset(seed=seed, specific_case_idx=idx)

#     # Cell and hidden state of the LSTM
#     lstm_states = None
#     num_envs    = 1
    
#     # Cell and hidden state of the LSTM
#     lstm_states = None
#     num_envs    = 1

#     # Episode start signals are used to reset the lstm states
#     episode_starts = np.ones((num_envs,), dtype=bool)
#     while True:
#         action, lstm_states = recurrent_ppo_model.predict(obs,
#                                                         state=lstm_states,
#                                                         episode_start=episode_starts,
#                                                         deterministic=True)
#         obs, rewards, terminated, truncated, info = env.step(action)
#         done = bool(terminated) or bool(truncated)

#         # Keep shape (1,), rather than changing to scalar bool
#         episode_starts = np.array([done], dtype=bool)
        
#         # Break the loop if it's either terminated or truncated
#         if done:
#             break
    
#     # Evaluate
#     if env.instance.all_ship_reaches_end_waypoint:
#         status_count["no_collision"]+=1
#     elif (env.instance.any_ship_collides and
#         not env.instance.termination_flags["collision_flags"]["OS0"]):
#         status_count["target_collision"]+=1
#     elif env.instance.termination_flags["collision_flags"]["OS0"]:
#         status_count["collision"]+=1
#     elif env.instance.termination_flags["nav_fail_flags"]["OS0"]:
#         status_count["nav_failure"]+=1
        
# # Compute the score
# each_flag_count = []
# for val in list(status_count.values()):
#     each_flag_count.append(val)
    
# no_collision        = each_flag_count[0] / np.sum(each_flag_count) * 100
# target_collision    = each_flag_count[1] / np.sum(each_flag_count) * 100
# collision           = each_flag_count[2] / np.sum(each_flag_count) * 100
# nav_failure         = each_flag_count[3] / np.sum(each_flag_count) * 100

# print(f"No Collision        : {no_collision} %")
# print(f"Target Collision    : {target_collision} %")
# print(f"Collision           : {collision} %")
# print(f"Nav Failure         : {nav_failure} %")