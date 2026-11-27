param(
    [ValidateSet('Live','Demo','Build','Test','FirmwareTest')][string]$Mode = 'Live',
    [string]$Port = '',
    [int]$Baud = 921600,
    [string]$BleAddress = 'C4:25:01:17:00:02',
    [string]$Gcc = '',
    [string]$Command = ''
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath $projectRoot
$pythonExe = Join-Path $projectRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $pythonExe)) {
    $pythonCommand = Get-Command python.exe -ErrorAction SilentlyContinue
    if (-not $pythonCommand) { $pythonCommand = Get-Command python -ErrorAction SilentlyContinue }
    if (-not $pythonCommand) { throw 'Python 3.10 or newer is required. Install Python or add it to PATH.' }
    $pythonExe = $pythonCommand.Source
}
foreach ($folder in @('logs','.runtime','.runtime\mpl')) {
    New-Item -ItemType Directory -Path (Join-Path $projectRoot $folder) -Force | Out-Null
}
$env:MPLCONFIGDIR = Join-Path $projectRoot '.runtime\mpl'
$env:PYTHONDONTWRITEBYTECODE = '1'
$env:PYTHONUTF8 = '1'
switch ($Mode) {
    'Live' {
        $logPath = Join-Path $projectRoot ('logs\live_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff') + '.bin')
        $monitorArgs = @('-B','pc\monitor.py','--log',$logPath)
        if ($Port) { $monitorArgs += @('--port',$Port,'--baud',"$Baud") }
        else { $monitorArgs += @('--ble',$BleAddress) }
        if ($Command) { $monitorArgs += @('--command',$Command) }
        & $pythonExe @monitorArgs
    }
    'Demo' { & $pythonExe -B pc\monitor.py --demo }
    'Build' {
        if ($Gcc) { & $pythonExe -B tools\build.py --gcc $Gcc }
        else { & $pythonExe -B tools\build.py }
    }
    'Test' { & $pythonExe -B tools\test_python.py }
    'FirmwareTest' { & $pythonExe -B tools\test_arm.py }
}
exit $LASTEXITCODE
