#!/usr/bin/env bash
# Builds the C++ FMUs on Linux and copies them to FMU_cpp/fmus/linux64/, in the same folders as the Python FMUs
# in FMUs/. Then run merge_fmus.py to make the FMUs in FMUs_cpp/.
# libstdc++ and libgcc are linked statically, so the FMUs only need a glibc as new as the one they were built with.
# Their symbols are hidden (--exclude-libs), so they do not mix with the libstdc++ of the program that loads the FMU.
# The build goes in build/linux/, so it does not mix with a Windows build in the same folder.

set -e
cd "$(dirname "$0")"

conan install . -pr:h default -pr:b default -s build_type=Release -s compiler.cppstd=20 \
    -c 'tools.cmake.cmake_layout:build_folder_vars=["settings.os"]' \
    -c 'tools.build:sharedlinkflags=["-static-libstdc++", "-static-libgcc", "-Wl,--exclude-libs,ALL"]'

cmake --preset conan-linux-release

cmake --build --preset conan-linux-release --target install

# Source folder in FMU_cpp/ -> output folder in FMU_cpp/fmus/linux64/
declare -A destination=(
    [simplified_ship]=simplified_ship
    [mission_manager]=simplified_ship
    [ship]=ship
    [env_loads]=env_loads
)
for folder in "${!destination[@]}"; do
    out="fmus/linux64/${destination[$folder]}"
    mkdir -p "$out"
    for model in "$folder"/*/; do
        model=$(basename "$model")
        cp "build/linux/Release/dist/fmus/$model.fmu" "$out"
        echo "FMU_cpp/$out/$model.fmu"
    done
done
