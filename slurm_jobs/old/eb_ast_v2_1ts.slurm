#!/bin/bash
#SBATCH --job-name=EBAST_v2_train_1ts
#SBATCH --account=share-ie-itk
#SBATCH --partition=GPUQ
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=16
#SBATCH --mem=128G
#SBATCH --time=48:00:00
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

python -u EBASTv2_train/argparse_train_vec_1ts.py --run_trained_episode=False --animate=False --plot=False --animation_show=False --animation_block=False --n_envs=16

echo "Finished at: $(date)"