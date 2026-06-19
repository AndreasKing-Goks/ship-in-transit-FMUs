#!/bin/bash
#SBATCH --job-name=EBAST_v2_train_2ts
#SBATCH --account=share-ie-itk
#SBATCH --partition=GPUQ
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=64
#SBATCH --mem=256G
#SBATCH --time=72:00:00
#SBATCH --output=/dev/null
#SBATCH --error=logs/%x_%j.err

echo "Running on node: $(hostname)"
echo "Job ID: $SLURM_JOB_ID"
echo "Started at: $(date)"

module purge
module load Anaconda3/2024.02-1
source $(conda info --base)/etc/profile.d/conda.sh
conda activate sit-cosim
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH

cd /cluster/work/arsitoru/projects/ship-in-transit-FMUs

which python
python --version

echo "GPU INFO:"
nvidia-smi

echo "Starting training..."

python -u EBASTv2_train/argparse_re-train_vec_2ts.py --results_ID=EB-ASTv2_train_2ts_continue_03_2026-06-13_22-30-22_69f2 --n_envs=32

echo "Finished at: $(date)"