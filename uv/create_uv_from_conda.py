"""
Function to create uv.toml file from environment.yml file.
Rerun this script when environment.yml changes.
"""

import os
import yaml

def create_uv_toml_from_env():
    """
    Creates a UV TOML file from an environment.yml file.
    Reads conda and pip dependencies and writes uv/uv.toml.
    """
    # Paths
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    env_file = os.path.join(project_root, "environment.yml")
    uv_dir = os.path.join(project_root, "uv")
    uv_file = os.path.join(uv_dir, "uv.toml")

    # Ensure uv directory exists
    os.makedirs(uv_dir, exist_ok=True)

    # Load environment.yml
    with open(env_file, "r", encoding="utf-8") as f:
        env_data = yaml.safe_load(f)

    dependencies = []

    # Parse conda dependencies
    for dep in env_data.get("dependencies", []):
        if isinstance(dep, str):
            # Standard conda dependency: package=version=build
            parts = dep.split("=")
            if len(parts) >= 2:
                pkg_name = parts[0].strip()
                pkg_version = parts[1].strip()
                dependencies.append((pkg_name, pkg_version))
        elif isinstance(dep, dict) and "pip" in dep:
            # Pip dependencies
            for pip_dep in dep["pip"]:
                if "==" in pip_dep:
                    pkg_name, pkg_version = pip_dep.split("==", 1)
                    dependencies.append((pkg_name.strip(), pkg_version.strip()))
                else:
                    dependencies.append((pip_dep.strip(), ""))

    # Write the UV TOML file
    with open(uv_file, "w", encoding="utf-8") as f:
        f.write("[dependencies]\n")
        for pkg_name, pkg_version in dependencies:
            if pkg_version:
                f.write(f'{pkg_name} = "{pkg_version}"\n')
            else:
                f.write(f'{pkg_name} = "*"\n')  # No version specified

    print(f"UV file '{uv_file}' generated with {len(dependencies)} dependencies.")

if __name__ == "__main__":
    create_uv_toml_from_env()