# Builds the C++ FMUs on Windows and copies them to FMUs_cpp/, in the same folders as the Python FMUs in FMUs/.

Set-Location $PSScriptRoot

conan install . -pr:h default -pr:b default -s build_type=Release -s compiler.cppstd=20
if ($LASTEXITCODE -ne 0) { throw "conan install failed" }

cmake --preset conan-default
if ($LASTEXITCODE -ne 0) { throw "cmake configure failed" }

cmake --build --preset conan-release --target install
if ($LASTEXITCODE -ne 0) { throw "cmake build failed" }

# Source folder in FMU_cpp/ -> output folder in FMUs_cpp/
$destination = @{
    "simplified_ship" = "simplified_ship"
    "mission_manager" = "simplified_ship"
    "ship"            = "ship"
    "env_loads"       = "env_loads"
}
foreach ($folder in $destination.Keys) {
    $out = Join-Path $PSScriptRoot "..\FMUs_cpp\$($destination[$folder])"
    New-Item -ItemType Directory -Force $out | Out-Null
    foreach ($model in Get-ChildItem -Directory $folder) {
        Copy-Item "build\dist\fmus\$($model.Name).fmu" $out
        Write-Output "FMUs_cpp\$($destination[$folder])\$($model.Name).fmu"
    }
}
