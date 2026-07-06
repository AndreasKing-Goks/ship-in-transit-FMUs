#!/bin/bash
#SBATCH --job-name=EBAST_v2_all
#SBATCH --account=share-ie-itk
#SBATCH --partition=GPUQ
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=32
#SBATCH --mem=256G
#SBATCH --time=12-00:00:00
#SBATCH --array=0-2
#SBATCH --output=logs/%x_%A_%a.out
#SBATCH --error=logs/%x_%A_%a.err

echo "Running on node: $(hostname)"
echo "Job ID: $SLURM_JOB_ID"
echo "Array Job ID: $SLURM_ARRAY_JOB_ID"
echo "Array Task ID: $SLURM_ARRAY_TASK_ID"
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

if [ "$SLURM_ARRAY_TASK_ID" -eq 0 ]; then
    ALG="ppo"
    SCRIPT="EBASTv2_train/argparse_train_vec_ppo.py"
elif [ "$SLURM_ARRAY_TASK_ID" -eq 1 ]; then
    ALG="rppo"
    SCRIPT="EBASTv2_train/argparse_train_vec_rppo.py"
elif [ "$SLURM_ARRAY_TASK_ID" -eq 2 ]; then
    ALG="sac"
    SCRIPT="EBASTv2_train/argparse_train_vec_sac.py"
else
    echo "Unknown array task ID: $SLURM_ARRAY_TASK_ID"
    exit 1
fi

echo "Starting $ALG training using $SCRIPT"

python -u "$SCRIPT" \
    --run_trained_episode=False \
    --animate=False \
    --plot=False \
    --animation_show=False \
    --animation_block=False \
    --n_envs=32

echo "Finished $ALG at: $(date)"