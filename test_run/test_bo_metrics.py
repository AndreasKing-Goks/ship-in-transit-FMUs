"""Focused source-function tests without FMU or Ax runtime dependencies."""

import argparse
import ast
from collections import defaultdict
from contextlib import redirect_stdout
import io
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]


def load_functions(path, namespace):
    # Importing bo_core would load the native simulator and the full Ax stack.
    tree = ast.parse(path.read_text(encoding="utf-8"))
    tree.body = [node for node in tree.body if isinstance(node, ast.FunctionDef)]
    exec(compile(tree, str(path), "exec"), namespace)
    return namespace


DESIGNS = {}
exec(compile((ROOT / "EBASTv2_core" / "reward_designs.py").read_text(encoding="utf-8"),
             "reward_designs.py", "exec"), DESIGNS)
CORE = load_functions(ROOT / "orchestrator" / "bo_core.py", {
    "np": np, "_final_distance_reward_func": DESIGNS["RewardDesign7"](100.0),
})


def flags(count=1):
    return {ship: {
        "collision": {"status": [False], "colliders": [None]},
        "grounding": [False], "navigation_failure": [False],
        "reaches_end_waypoint": [False], "outside_horizon": [False],
    } for ship in ["OS0"] + [f"TS{k}" for k in range(1, count + 1)]}


def set_event(info, ship, event, colliders=None):
    if event == "collision":
        info[ship][event] = {"status": [True], "colliders": [colliders or ["OTHER"]]}
    else:
        info[ship][event] = [True]


class Trial:
    def __init__(self, distances=((1000.0, 1000.0),), event=None, skip_map=False):
        self.stop_info = flags(len(distances))
        if event:
            set_event(self.stop_info, *event)
        self.ship_configs = [{"id": ship, "spawn": {"start_time": 0.0}}
                             for ship in self.stop_info]
        self.skip_map_evaluation = skip_map
        self.time = self.stopTime = 15_000_000_000
        self.times = np.array([5.0, 15.0])
        self.positions = {"OS0": np.zeros(2)}
        self.positions.update({f"TS{k}": np.asarray(values, dtype=float)
                               for k, values in enumerate(distances, 1)})

    def get_ship_timeseries(self, ship, variable):
        assert variable in ("north", "east"), "COLAV distance must not be used"
        values = self.positions[ship] if variable == "north" else np.zeros(2)
        return self.times, np.arange(2), values

    def ship_slave(self, ship, block):
        return f"{ship}__{block}"

    def GetLastValue(self, slaveName, slaveVar):
        ship = slaveName.split("__")[0]
        return self.positions[ship][-1] if slaveVar == "north" else 0.0


class MetricsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        designs = subprocess.check_output(
            ["git", "show", "3ce5ac7:EBASTv2_core/reward_designs.py"], cwd=ROOT,
            text=True, encoding="utf-8",
        )
        reward = subprocess.check_output(
            ["git", "show", "3ce5ac7:EBASTv2_core/reward_function.py"], cwd=ROOT,
            text=True, encoding="utf-8",
        )
        cls.ast_reward = {}
        exec(compile(designs, "ast_reward_designs.py", "exec"), cls.ast_reward)
        tree = ast.parse(reward)
        tree.body = [node for node in tree.body if not (
            isinstance(node, ast.ImportFrom) and node.module == "EBASTv2_core.reward_designs")]
        exec(compile(tree, "ast_reward_function.py", "exec"), cls.ast_reward)

    def evaluate(self, trial):
        return CORE["compute_trial_metrics"](trial, len(trial.ship_configs) - 1)

    def test_curve_matches_latest_ast(self):
        for distance in (0.0, 99.0, 100.0, 101.0, 1000.0, 5000.0):
            self.assertAlmostEqual(CORE["_final_distance_reward_func"](distance),
                                   self.ast_reward["nearest_distance_reward_func"](distance))

    def test_terminal_component_matches_latest_ast_for_all_events(self):
        for ship, event in (("OS0", "collision"), ("OS0", "grounding"),
                            ("OS0", "navigation_failure"), ("OS0", "reaches_end_waypoint"),
                            ("TS1", "collision"), ("TS1", "grounding"),
                            ("TS1", "navigation_failure"), ("TS1", "reaches_end_waypoint")):
            with self.subTest(ship=ship, event=event):
                trial = Trial(event=(ship, event))
                parts = defaultdict(list)
                observation = {
                    "own_ship_pos": np.zeros(3), "own_ship_forward_speed": np.array([0.0]),
                    "rel_tar_ships_pos": np.array([1000.0, 0.0, 0.0]),
                    "tar_ships_forward_speed": np.array([8.0]),
                    "remaining_requests": np.array([4.0]),
                }
                args = ("OS0", ["TS1"], [1], trial.stop_info, 1, parts, False,
                        ["TS1"], {"TS1": 1000.0}, {"max": np.array([4.0])},
                        [60.0], True, [0.0], [0.0], [True], [180.0])
                self.ast_reward["compute_reward"](observation, args)
                result = self.evaluate(trial)
                self.assertAlmostEqual(result["terminal_reward"], parts["termination_reward"][-1])
                self.assertEqual(result["objective"], -result["terminal_reward"])

    def test_final_distance_mean_not_worst_or_minimum_distance(self):
        trial = Trial(((100.0, 1000.0), (200.0, 5000.0)), ("OS0", "reaches_end_waypoint"))
        result = self.evaluate(trial)
        expected = (-np.log(901.0) - np.log(4901.0)) / 2
        self.assertAlmostEqual(result["terminal_reward"], expected)
        self.assertEqual(result["min_dist"], 100.0)
        self.assertEqual(result["final_dist_tar_2"], 5000.0)

    def test_positive_failure_has_no_distance_bonus(self):
        result = self.evaluate(Trial(((100.0, 100.0),), ("OS0", "collision")))
        self.assertEqual(result["danger_score"], 0.0)
        self.assertEqual(result["objective"], 0.0)

    def test_negative_outcome_overrides_simultaneous_own_failure(self):
        trial = Trial(event=("OS0", "collision"))
        set_event(trial.stop_info, "TS1", "reaches_end_waypoint")
        result = self.evaluate(trial)
        self.assertAlmostEqual(result["terminal_reward"], -np.log(901.0))
        self.assertEqual(result["termination_reason"], "negative_termination")

    def test_collision_with_own_ship_is_not_invalid_target_collision(self):
        trial = Trial(event=("OS0", "collision"))
        set_event(trial.stop_info, "TS1", "collision", ["OS0"])
        result = self.evaluate(trial)
        self.assertFalse(result["tar_ships_collision"])
        self.assertEqual(result["terminal_reward"], 0.0)

    def test_timeout_and_map_exit_are_penalized(self):
        for event, reason in ((None, "time_limit"), (("TS1", "outside_horizon"), "outside_horizon")):
            result = self.evaluate(Trial(event=event))
            self.assertAlmostEqual(result["terminal_reward"], -np.log(901.0))
            self.assertTrue(result["unfinished"])
            self.assertEqual(result["termination_reason"], reason)

    def test_map_flags_ignored_when_disabled(self):
        trial = Trial(event=("OS0", "grounding"), skip_map=True)
        set_event(trial.stop_info, "TS1", "grounding")
        set_event(trial.stop_info, "TS1", "outside_horizon")
        result = self.evaluate(trial)
        self.assertFalse(result["own_ship_grounding"])
        self.assertFalse(result["tar_ships_grounding"])
        self.assertEqual(result["termination_reason"], "time_limit")

    def test_initial_and_pre_spawn_samples_excluded_from_diagnostics(self):
        trial = Trial(((0.0, 1000.0),))
        trial.times = np.array([0.0, 15.0])
        self.assertEqual(self.evaluate(trial)["min_dist"], 1000.0)
        trial.times = np.array([5.0, 15.0])
        trial.ship_configs[1]["spawn"]["start_time"] = 10.0
        self.assertEqual(self.evaluate(trial)["min_dist"], 1000.0)

    def test_bad_inputs_raise_instead_of_producing_safe_scores(self):
        invalid = Trial(((np.nan, 1000.0),))
        missing = Trial()
        missing.stop_info["OS0"]["navigation_failure"] = []
        unknown = Trial()
        unknown.time = 0
        unspawned = Trial()
        unspawned.ship_configs[1]["spawn"]["start_time"] = 20.0
        for trial in (invalid, missing, unknown, unspawned):
            with self.subTest(trial=trial), self.assertRaises(ValueError):
                self.evaluate(trial)
        with self.assertRaises(ValueError):
            CORE["compute_trial_metrics"](Trial(), 2)

    def test_misaligned_histories_raise(self):
        trial = Trial()
        original = trial.get_ship_timeseries
        trial.get_ship_timeseries = lambda ship, var: (
            (np.array([6.0, 15.0]), np.arange(2), np.zeros(2))
            if ship == "TS1" else original(ship, var))
        with self.assertRaises(ValueError):
            self.evaluate(trial)


class FakeAxClient:
    def __init__(self, **kwargs):
        self.data = {}
        self.updates = []
        self.experiment = SimpleNamespace(trials={})

    def create_experiment(self, **kwargs):
        pass

    def get_next_trial(self):
        index = len(self.experiment.trials)
        self.experiment.trials[index] = SimpleNamespace(generator_runs=[
            SimpleNamespace(_generation_node_name="BoTorch")])
        return {"x": index}, index

    def complete_trial(self, trial_index, raw_data):
        self.data[trial_index] = raw_data["objective"][0]

    def get_trial(self, trial_index):
        def update_trial_data(raw_data):
            self.updates.append(trial_index)
            self.complete_trial(trial_index, raw_data)
        return SimpleNamespace(update_trial_data=update_trial_data)

    def get_best_trial(self, trial_indices, use_model_predictions):
        self.feasible_indices = trial_indices
        index = min(trial_indices, key=self.data.get)
        return index, {"x": index}, None


class OptimizationTests(unittest.TestCase):
    def test_penalties_rescale_and_best_selection_excludes_infeasible_trials(self):
        calls = iter([ValueError("unplaceable"), 30.0, ValueError("unplaceable"), 50.0])

        def evaluate(parameters):
            value = next(calls)
            if isinstance(value, Exception):
                raise value
            return {"objective": (value, None)}, {"objective": value}

        namespace = dict(CORE, AxClient=FakeAxClient,
                         GenerationStep=lambda **kwargs: kwargs,
                         GenerationStrategy=lambda **kwargs: kwargs,
                         Generators=SimpleNamespace(SOBOL="Sobol", BOTORCH_MODULAR="BoTorch"),
                         ObjectiveProperties=lambda **kwargs: kwargs,
                         traceback=SimpleNamespace(format_exc=lambda: "evaluation failed"))
        namespace = load_functions(ROOT / "orchestrator" / "bo_core.py", namespace)
        with redirect_stdout(io.StringIO()):
            client, best, history = namespace["run_ax_optimization"](
                "test", [], evaluate, total_trials=2, num_sobol_trials=0)
        self.assertEqual(client.data, {0: 55.0, 1: 30.0, 2: 55.0, 3: 50.0})
        self.assertEqual(best, {"x": 1})
        self.assertEqual(client.feasible_indices, [1, 3])
        self.assertEqual(history[0]["metrics"]["objective"], 55.0)
        self.assertEqual(history[2]["metrics"]["objective"], 55.0)

    def test_replay_best_uses_feasible_minimum_objective(self):
        df = pd.DataFrame({"trial": [20, 21, 22], "status": ["FAILED", "COMPLETED", "COMPLETED"],
                           "objective": [-100.0, 8.0, 0.0], "danger_score": [100.0, -8.0, 0.0]},
                          index=[7, 9, 11])
        captured = []
        namespace = load_functions(ROOT / "orchestrator" / "bo_replay_trial.py", {
            "np": np, "pd": pd, "argparse": argparse, "Path": Path,
        })
        namespace.update({
            "get_scenario_info": lambda *args: {"config_path": "config", "encounter_settings_path": "settings"},
            "load_csv_and_json": lambda paths: (df, {}),
            "parse_encounters_from_results": lambda *args: [],
        })
        namespace["replay_trial"] = lambda index, *args, **kwargs: captured.append(index)
        with patch.object(sys, "argv", ["replay", "--scenario", "config", "--best"]), redirect_stdout(io.StringIO()):
            namespace["main"]()
        self.assertEqual(captured, [2])


if __name__ == "__main__":
    unittest.main()
