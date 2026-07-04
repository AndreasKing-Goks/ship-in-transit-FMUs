from pathlib import Path
import sys

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from orchestrator.scenario_config import generate_spawn_request_bank, load_spawn_requests_bank_path

config_path                 = ROOT / "EBASTv2_train" / "EBASTv2_train_2.yaml"
encounter_settings_path     = ROOT / "EBASTv2_train" / "encounter_settings.json"
spawn_requests_bank_path    = ROOT / "pre_generated_spawn_requests_bank" / "spawn_request_bank_1000.pkl"
n_cases                     = 10
training_case_ratio         = 0.9

spawn_requests_bank_path = generate_spawn_request_bank(
    ROOT=ROOT,
    config_path=config_path,
    encounter_settings_path=encounter_settings_path,
    spawn_requests_bank_path=spawn_requests_bank_path,
    n_cases=n_cases,
    training_case_ratio=training_case_ratio,
    overwrite=True
)
spawn_requests_bank = load_spawn_requests_bank_path(spawn_requests_bank_path)