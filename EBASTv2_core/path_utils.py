from pathlib import Path
import re
import datetime
import uuid

'''
Helper function for getting the data path
'''

def get_RL_model_path(root: Path, model_name: str, unique: bool = True, save_anim_filename: str = None):
    """
    Generate model and log paths with unique timestamp (and short UUID) suffix.
    
    Example:
        model_path, log_path = get_trained_model_path(ROOT, "AST-train")
        
        # Returns something like:
        # model_path = "...trained_model/AST-train_2025-11-09_18-25-03_ab12/model"
        # log_path   = "...trained_model/AST-train_2025-11-09_18-25-03_ab12/log"
        # tb_path    = "...trained_model/AST-train_2025-11-09_18-25-03_ab12/tb"
    """
    # Base directory
    base_dir = Path(root) / "EBASTv2_train" / "trained_model"
    base_dir.mkdir(parents=True, exist_ok=True)

    # Create unique suffix (timestamp + short UUID)
    if unique:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        short_id = uuid.uuid4().hex[:4]  # short unique string
        model_name_unique = f"{model_name}_{timestamp}_{short_id}"
    else:
        model_name_unique = model_name

    # Full paths
    model_dir = base_dir / model_name_unique / "model"
    model_dir.mkdir(parents=True, exist_ok=True)
    model_path = str(model_dir / "model.zip")
    
    checkpoint_dir = base_dir / model_name_unique / "checkpoints"

    log_dir = base_dir / model_name_unique / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    
    train_args_log_path = str(log_dir / "train_args.txt")
    episode_log_path    = str(log_dir / "episode_recap.txt")

    tb_dir = base_dir / model_name_unique / "tb"
    tb_dir.mkdir(parents=True, exist_ok=True)
    tb_path = str(tb_dir)
    
    if save_anim_filename is not None:
        saved_anim_dir = base_dir / model_name_unique / "saved_animation"

        # Create directory automatically if it does not exist
        saved_anim_dir.mkdir(parents=True, exist_ok=True)

        saved_anim_path = str(saved_anim_dir / save_anim_filename)
    else:
        saved_anim_path = None

    return model_path, train_args_log_path, episode_log_path, tb_path, saved_anim_path, checkpoint_dir

def get_next_continue_name(results_ID: str) -> str:
    """
        Helper function for the first re-training attempt
    """
    # re.search(pattern, string)
    match = re.search(r"_continue_(\d+)$", results_ID)

    if match:
        current_number = int(match.group(1))
        next_number = current_number + 1

        # re.sub(pattern, replacement, string)
        return re.sub(
            r"_continue_\d+$",
            f"_continue_{next_number:02d}",
            results_ID,
        )

    return f"{results_ID}_continue_01"

def get_re_trained_RL_model_path(root: Path, results_ID: str, unique: bool = True, save_anim_filename: str = None):
    """
    Use the model path name that will be used for retraining, and modify to create new directories
    
    - "wxyz" is a unique string
    
    Example 1:
        old run:
        EB-ASTv2_train_yyyy-mm-dd_hh-mm-ss_wxyz/

        continued run:
        EB-ASTv2_train_yyyy-mm-dd_hh-mm-ss_wxyz_continue_01/
        
    Example 2:
        old run:
        EB-ASTv2_train_yyyy-mm-dd_hh-mm-ss_wxyz_continue_01/

        continued run:
        EB-ASTv2_train_yyyy-mm-dd_hh-mm-ss_wxyz_continue_02/
    """
    # Base directory
    base_dir = Path(root) / "EBASTv2_train" / "trained_model"
    base_dir.mkdir(parents=True, exist_ok=True)
    
    # Retrieve the train results ID and modify it
    if unique:
        model_name_unique   = get_next_continue_name(results_ID)
    else:
        model_name_unique   = f"{results_ID}_continue"

    # Full paths
    model_dir = base_dir / model_name_unique / "model"
    model_dir.mkdir(parents=True, exist_ok=True)
    model_path = str(model_dir / "model.zip")
    
    checkpoint_dir = base_dir / model_name_unique / "checkpoints"
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

    log_dir = base_dir / model_name_unique / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    
    train_args_log_path = str(log_dir / "train_args.txt")
    episode_log_path    = str(log_dir / "episode_recap.txt")

    tb_dir = base_dir / model_name_unique / "tb"
    tb_dir.mkdir(parents=True, exist_ok=True)
    tb_path = str(tb_dir)
    
    if save_anim_filename is not None:
        saved_anim_dir = base_dir / model_name_unique / "saved_animation"

        # Create directory automatically if it does not exist
        saved_anim_dir.mkdir(parents=True, exist_ok=True)

        saved_anim_path = str(saved_anim_dir / save_anim_filename)
    else:
        saved_anim_path = None

    return model_path, train_args_log_path, episode_log_path, tb_path, saved_anim_path, checkpoint_dir