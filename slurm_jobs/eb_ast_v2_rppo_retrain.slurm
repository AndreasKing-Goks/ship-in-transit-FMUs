#!/bin/bash
#SBATCH --job-name=EBAST_v2_retrain_rppo
#SBATCH --account=share-ie-itk
#SBATCH --partition=GPUQ
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=64
#SBATCH --mem=256G
#SBATCH --time=12-00:00:00
#SBATCH --output=logs/rppo/%x_%A_%a.out
#SBATCH --error=logs/rppo/%x_%j.err

set -euo pipefail

echo "Running on node: $(hostname)"
echo "Job ID: $SLURM_JOB_ID"
echo "Started at: $(date)"

module purge
module load Anaconda3/2024.02-1
source $(conda info --base)/etc/profile.d/conda.sh
conda activate sit-cosim
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"

cd /cluster/work/arsitoru/projects/ship-in-transit-FMUs

which python
python --version

echo "GPU INFO:"
nvidia-smi

echo "Starting training..."

python -u EBASTv2_train/argparse_retrain_vec_rppo.py --use_fmpy=False --n_envs=64

echo "Finished at: $(date)"