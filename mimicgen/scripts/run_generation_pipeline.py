
from __future__ import annotations

import argparse
import glob
import os
import shutil
import subprocess
import sys
from pathlib import Path

SHARED_DATA_PTH = Path("/mnt/nfs_client/robocasa/datasets/v0.5")
LOCAL_DATA_PTH = Path("/mnt/data1/abhim/robocasa/datasets/v0.5")
MG_PATH = Path("/home/abhim/robocasa/mimicgen-dev")
ROBOCASA_PATH = Path("/home/abhim/robocasa/robocasa-dev")


THREAD_LIMIT_VARS = {
    "OMP_NUM_THREADS": "1",
    "MPI_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
}


def build_env(device_num: int) -> dict[str, str]:
    """Return a copy of the current environment with GPU & thread limits set."""
    env = os.environ.copy()
    env.update(THREAD_LIMIT_VARS)
    env["CUDA_VISIBLE_DEVICES"] = str(device_num)
    env["EGL_DEVICE_ID"] = str(device_num)
    return env



def copy_file(src: Path, dst: Path, overwrite: bool = False) -> None:
    """Copy *file* ``src`` → ``dst`` creating parent dirs. Skip if exists and
    *overwrite* is false."""
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists() and not overwrite:
        print("[skip] %s already exists", dst)
        return
    print("Copying %s → %s", src, dst)
    shutil.copy2(src, dst)


def copy_tree(src: Path, dst: Path, overwrite: bool = False) -> None:
    """Copy directory tree ``src`` → ``dst``. Uses ``dirs_exist_ok`` on py≥3.8."""
    if dst.exists() and overwrite:
        shutil.rmtree(dst)
    print("Sync %s → %s", src, dst)
    shutil.copytree(src, dst, dirs_exist_ok=True)



def run_pipeline(
    args,
    dataset: str,
    split: str,
    device_num: int,
    num_gen_procs: int,
    num_extraction_procs: int,
    overwrite: bool,
    shared_root: Path = SHARED_DATA_PTH,
    local_root: Path = LOCAL_DATA_PTH,
    mg_path: Path = MG_PATH,
    robocasa_path: Path = ROBOCASA_PATH,
) -> None:

    part1 = args.part1
    part2 = args.part2
    part3 = args.part3
    env = build_env(device_num)
    num_demos = args.num_demos

    # ---------------------------------------------------------------------
    # 1) Locate the source dataset on the shared drive & copy locally
    # ---------------------------------------------------------------------

    shared_parent = shared_root / split
    pattern = f"{shared_parent}/*/{dataset}/**/demo.hdf5"
    shared_datasets = glob.glob(pattern, recursive=True)
    assert (
        len(shared_datasets) == 1
    ), f"Expected exactly one dataset for '{dataset}', got: {shared_datasets}"

    src_path = Path(shared_datasets[0])
    rel_path = src_path.relative_to(shared_root)
    local_dataset_path = local_root / rel_path
    if part1:
        print(f"copying dataset from {src_path} to {local_dataset_path}")
        copy_file(src_path, local_dataset_path, overwrite)

    # ---------------------------------------------------------------------
    # 2) Prepare source dataset (MimicGen)
    # ---------------------------------------------------------------------
    if part1:
        subprocess.run(
                [
                    "python",
                    mg_path / "mimicgen/scripts/prepare_src_dataset.py",
                    "--dataset",
                    str(local_dataset_path),
                ],
                check=True,
                env=env,
            )

    # ---------------------------------------------------------------------
    # 3) Generation
    # ---------------------------------------------------------------------

    cfg_pattern = mg_path / f"mimicgen/exps/templates/robocasa/**/{dataset}.json"
    cfg_list = glob.glob(str(cfg_pattern), recursive=True)
    assert (
        len(cfg_list) == 1
    ), f"Expected one config for '{dataset}', found: {cfg_list}"
    config_path = Path(cfg_list[0])

    gen_out_root = local_dataset_path.parents[1] / "mg"
    gen_cmd = [
        "python",
        str(mg_path / "mimicgen/scripts/generate_dataset_multicore.py"),
        "--source",
        str(local_dataset_path),
        "--config",
        str(config_path),
        "--folder",
        str(gen_out_root),
        "--num_procs",
        str(num_gen_procs),
        "--num_demos",
        str(num_demos)
    ]
    if part1:
        subprocess.run(gen_cmd, check=True, env=env)
    

    # ---------------------------------------------------------------------
    # 4) Extraction (Robocasa)
    # ---------------------------------------------------------------------

    gen_ds_list = glob.glob(str(gen_out_root / "**/demo.hdf5"), recursive=True)
    assert (
        len(gen_ds_list) == 1
    ), f"Expected one generated dataset, found: {gen_ds_list}"
    generated_dataset = Path(gen_ds_list[0])

    extract_cmd = [
        "python",
        robocasa_path
        / "robocasa/scripts/dataset_scripts/dataset_states_to_obs.py",
        "--dataset",
        str(generated_dataset),
        "--num_procs",
        str(num_extraction_procs),
    ]
    if part2:
        subprocess.run(extract_cmd, check=True, env=env)

    # ---------------------------------------------------------------------
    # 5) Sync generated dataset back to shared location
    # ---------------------------------------------------------------------

    shared_dst_root = Path(
        glob.glob(f"{shared_parent}/*/{dataset}/")[0]
    )  

    if part3:
        copy_tree(gen_out_root, shared_dst_root / "mg", overwrite)

    print("Pipeline complete")


# -----------------------------------------------------------------------------
# CLI entrypoint
# -----------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--dataset", required=True)

    p.add_argument("--split", choices=["train", "test"], default="train", help="Dataset split")
    p.add_argument("-d", "--device-num", type=int, required=True)
    p.add_argument("--num_demos", type=int, default=1000, help="Processes for generation stage")
    p.add_argument("--num-gen-procs", type=int, default=15, help="Processes for generation stage")
    p.add_argument("--num-extraction-procs", type=int, default=10, help="Processes for extraction stage")
    p.add_argument("--overwrite", action="store_true", help="Overwrite existing local / shared outputs")
    p.add_argument("--part1", action="store_true")
    p.add_argument("--part2", action="store_true")
    p.add_argument("--part3", action="store_true")

    # Path overrides (rarely needed)
    p.add_argument("--shared-root", type=Path, default=SHARED_DATA_PTH)
    p.add_argument("--local-root", type=Path, default=LOCAL_DATA_PTH)
    p.add_argument("--mg-path", type=Path, default=MG_PATH)
    p.add_argument("--robocasa-path", type=Path, default=ROBOCASA_PATH)

    args = p.parse_args()
    return args


def main() -> None:
    args = parse_args()
    run_pipeline(
        args,
        dataset=args.dataset,
        split=args.split,
        device_num=args.device_num,
        num_gen_procs=args.num_gen_procs,
        num_extraction_procs=args.num_extraction_procs,
        overwrite=args.overwrite,
        shared_root=args.shared_root,
        local_root=args.local_root,
        mg_path=args.mg_path,
        robocasa_path=args.robocasa_path,
    )


if __name__ == "__main__":
    main()
