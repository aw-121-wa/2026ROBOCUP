$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
cmake --preset Route-Current
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
cmake --build --preset Route-Current -j 8
exit $LASTEXITCODE
