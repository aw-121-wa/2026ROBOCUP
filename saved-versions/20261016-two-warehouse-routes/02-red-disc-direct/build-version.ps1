$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
cmake --preset Route-Red-Direct
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
cmake --build --preset Route-Red-Direct -j 8
exit $LASTEXITCODE
