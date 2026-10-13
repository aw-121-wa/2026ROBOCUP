param(
    [Parameter(Mandatory=$true)][ValidateSet('destack','bottom')][string]$Mode,
    [string]$RdkAddress='192.168.128.10',
    [int]$Count=15
)
$ErrorActionPreference='Stop'
if ($Count -lt 1 -or $Count -gt 100) { throw 'Count must be 1..100' }
if ($RdkAddress -notmatch '^[a-zA-Z0-9.-]+$') { throw 'Invalid RDK address' }
$repo=Split-Path $PSScriptRoot -Parent
Set-Location $repo
$key=Join-Path $repo 'build/rdk_deploy_key'
$opts=@('-o','ProxyCommand=none','-o','ProxyJump=none','-o','HostKeyAlias=10.232.199.232','-o','BatchMode=yes','-o','ConnectTimeout=5','-i',$key)
$run='warehouse-'+$Mode+'-'+(Get-Date -Format 'yyyyMMdd-HHmmss')
$remote='/tmp/'+$run
$local=Join-Path $repo ('build/captures/'+$run)
New-Item -ItemType Directory -Force -Path $local | Out-Null
function Remote([string]$Command) {
    & ssh @opts "root@$RdkAddress" $Command
    if ($LASTEXITCODE -ne 0) { throw "RDK command failed: $Command" }
}
& scp -O @opts (Join-Path $PSScriptRoot 'capture_warehouse_remote.py') "root@${RdkAddress}:/tmp/capture_warehouse_remote.py"
if ($LASTEXITCODE -ne 0) { throw 'Upload failed' }
Read-Host 'Keep chassis stationary and clear the arm travel area. Press Enter to begin'
try {
    Remote 'systemctl stop rdk-disc.service'
    $rows=if ($Mode -eq 'destack') { @(3,2,1) } else { @(0) }
    foreach ($row in $rows) {
        if ($Mode -eq 'destack') {
            $group=121-3*$row
            Remote "cd /home/sunrise/licang_vision && PYTHONPATH=.:tools python3 /tmp/capture_warehouse_remote.py --mode destack --row $row --digit 1 --out $remote --pose-only"
        }
        foreach ($digit in @(1,2,3)) {
            Read-Host "Mode=$Mode row=$row : place digit $digit in view, keep other digits out of this row, then press Enter"
            Remote "cd /home/sunrise/licang_vision && PYTHONPATH=.:tools python3 /tmp/capture_warehouse_remote.py --mode $Mode --row $row --digit $digit --count $Count --out $remote"
            $part=if ($Mode -eq 'destack') { "row$row" } else { 'bottom' }
            $target=Join-Path $local $part
            New-Item -ItemType Directory -Force -Path $target | Out-Null
            & scp -O -r @opts "root@${RdkAddress}:$remote/$part/$digit" $target
            if ($LASTEXITCODE -ne 0) { throw 'Download failed' }
            Write-Host "Saved locally: $local"
        }
    }
} finally {
    & ssh @opts "root@$RdkAddress" 'systemctl start rdk-disc.service; systemctl is-active rdk-disc.service'
}
Write-Host "Capture complete: $local"
