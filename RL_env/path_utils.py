from pathlib import Path
import re
import datetime
import uuid

'''
Helper function for getting the data path
'''

def get_RL_model_path(root: Path, model_name: str, unique: bool = True):
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
    model_path = str(base_dir / model_name_unique / "model")
    log_path   = str(base_dir / model_name_unique / "log")
    tb_path   = str(base_dir / model_name_unique / "tb")

    return model_path, log_path, tb_path