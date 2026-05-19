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
        # model_path = ".../trained_model/AST-train_2025-11-09_18-25-03_ab12/model"
        # log_path   = ".../trained_model/AST-train_2025-11-09_18-25-03_ab12/log"
        # tb_path   = ".../trained_model/AST-train_2025-11-09_18-25-03_ab12/tb"
    """
    # Base directory
    base_dir = Path(root) / "train_ast" / "trained_model"

    # Create unique suffix (timestamp + short UUID)
    if unique:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        short_id = uuid.uuid4().hex[:4]  # short unique string
        model_name_unique = f"{model_name}_{timestamp}_{short_id}"
    else:
        model_name_unique = model_name

    # Full paths
    model_dir      = str(base_dir / model_name_unique / "model")
    model_dir.mkdir(parents=True, exist_ok=True)
    model_path = str(model_dir)
    
    log_dir        = str(base_dir / model_name_unique / "log")
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path   = str(log_dir)
    
    tb_dir         = str(base_dir / model_name_unique / "tb")
    tb_dir.mkdir(parents=True, exist_ok=True)
    tb_path    = str(tb_dir)
    
    if save_anim_filename is not None:
        saved_anim_dir = base_dir / model_name_unique / "saved_animation"

        # Create directory automatically if it does not exist
        saved_anim_dir.mkdir(parents=True, exist_ok=True)

        saved_anim_path = str(saved_anim_dir / save_anim_filename)
    else:
        saved_anim_path = None

    return model_path, log_path, tb_path, saved_anim_path