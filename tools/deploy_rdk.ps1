param(
    [string]$RdkAddress = "10.205.175.232",
    [string]$KeyPath = "build/rdk_deploy_key"
)
$ErrorActionPreference = "Stop"
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    if (!(Test-Path -LiteralPath $KeyPath)) { throw "SSH key missing: $KeyPath" }
    python tools/package_warehouse_deploy.py
    if ($LASTEXITCODE -ne 0) { throw "Packaging failed" }
    $sshOptions = @("-o", "ProxyCommand=none", "-o", "ProxyJump=none",
        "-o", "HostKeyAlias=10.232.199.232", "-o", "BatchMode=yes",
        "-o", "ConnectTimeout=8", "-i", $KeyPath)
    scp -O @sshOptions build/warehouse-continuous-deploy.tar.gz build/warehouse-deploy-manifest.json tools/deploy_warehouse.sh "root@${RdkAddress}:/tmp/"
    if ($LASTEXITCODE -ne 0) { throw "Upload failed; existing RDK service was not changed" }
    ssh @sshOptions "root@$RdkAddress" "bash /tmp/deploy_warehouse.sh"
    if ($LASTEXITCODE -ne 0) { throw "Deployment failed; inspect RDK service and backup" }
    ssh @sshOptions "root@$RdkAddress" "systemctl show rdk-disc.service -p ActiveState -p SubState -p NRestarts; journalctl -u rdk-disc.service -n 8 --no-pager"
    if ($LASTEXITCODE -ne 0) { throw "Post-deployment check failed" }
} finally { Pop-Location }
