"""
Combine the FMUs of the Windows and Linux builds into FMUs with binaries for both, in FMUs_cpp/.

build_win.ps1 and build_linux.sh put their FMUs in FMU_cpp/fmus/<platform>/. Then run:

    python FMU_cpp/merge_fmus.py

This combines all platform folders in FMU_cpp/fmus/. Other folders can be given instead:

    python FMU_cpp/merge_fmus.py <platform folder> <platform folder> ...

Each FMU gets the binaries of every platform folder, and everything else (modelDescription.xml, documentation,
...) from the first folder. The script stops if an FMU is missing in one of the folders, if the modelDescription.xml
files are not the same (another GUID means other sources), or if two folders have binaries for the same platform.
"""

from pathlib import Path
import sys
import zipfile

HERE = Path(__file__).resolve().parent
PLATFORM_FMUS = HERE / "fmus"
FMUS_CPP = HERE.parent / "FMUs_cpp"


def platform_of(name):
    """binaries/<platform>/... -> <platform>, None for other files."""
    parts = name.split("/")
    if len(parts) > 2 and parts[0] == "binaries":
        return parts[1]
    return None


def model_description(zip_file):
    # Line endings may differ between the platforms
    return zip_file.read("modelDescription.xml").replace(b"\r\n", b"\n")


def check(fmu_rel, input_dirs):
    """Stop if the FMUs can not be combined. Returns the platforms."""
    platforms = {}  # platform -> input folder
    for input_dir in input_dirs:
        with zipfile.ZipFile(input_dirs[0] / fmu_rel) as first, zipfile.ZipFile(input_dir / fmu_rel) as fmu:
            if model_description(fmu) != model_description(first):
                sys.exit(f"{fmu_rel}: modelDescription.xml in {input_dir} is not the same as in {input_dirs[0]}")
            for platform in {platform_of(name) for name in fmu.namelist()} - {None}:
                if platform in platforms:
                    sys.exit(f"{fmu_rel}: binaries/{platform} is in both {platforms[platform]} and {input_dir}")
                platforms[platform] = input_dir
    return sorted(platforms)


def merge(fmu_rel, input_dirs, out_path):
    """Write one FMU with the binaries from every input, and the rest from the first."""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(out_path, "w", zipfile.ZIP_DEFLATED) as out:
        for input_dir in input_dirs:
            first = input_dir == input_dirs[0]
            with zipfile.ZipFile(input_dir / fmu_rel) as fmu:
                for info in fmu.infolist():
                    if platform_of(info.filename) is not None or (first and not info.filename.startswith("binaries/")):
                        out.writestr(info, fmu.read(info))


if __name__ == "__main__":
    input_dirs = [Path(p) for p in sys.argv[1:]] or sorted(d for d in PLATFORM_FMUS.glob("*") if d.is_dir())
    if not input_dirs:
        sys.exit(f"No platform folders in {PLATFORM_FMUS}. Run build_win.ps1 or build_linux.sh first.")

    fmus = sorted(p.relative_to(input_dirs[0]) for p in input_dirs[0].glob("*/*.fmu"))
    for input_dir in input_dirs[1:]:
        others = sorted(p.relative_to(input_dir) for p in input_dir.glob("*/*.fmu"))
        if others != fmus:
            sys.exit(f"{input_dir} does not have the same FMUs as {input_dirs[0]}")

    # Check all FMUs before writing any, so FMUs_cpp/ is not left half updated
    platforms = {fmu_rel: check(fmu_rel, input_dirs) for fmu_rel in fmus}

    for fmu_rel in fmus:
        merge(fmu_rel, input_dirs, FMUS_CPP / fmu_rel)
        print(f"FMUs_cpp/{fmu_rel.as_posix()}: {', '.join(platforms[fmu_rel])}")
