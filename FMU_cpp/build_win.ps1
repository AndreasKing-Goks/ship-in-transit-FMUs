# Builds the C++ FMUs on Windows and copies them to FMUs_cpp/simplified_ship.

Set-Location $PSScriptRoot

conan install . -pr:h default -pr:b default -s build_type=Release -s compiler.cppstd=20
if ($LASTEXITCODE -ne 0) { throw "conan install failed" }

cmake --preset conan-default
if ($LASTEXITCODE -ne 0) { throw "cmake configure failed" }

cmake --build --preset conan-release --target install
if ($LASTEXITCODE -ne 0) { throw "cmake build failed" }

$out = Join-Path $PSScriptRoot "..\FMUs_cpp\simplified_ship"
New-Item -ItemType Directory -Force $out | Out-Null
Copy-Item "build\dist\fmus\*.fmu" $out
Get-ChildItem $out
