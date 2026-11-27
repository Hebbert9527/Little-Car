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
foreach ($folder in @('logs','.runtime','.runtime\mpl','.runtime\tmp','.runtime\pip-cache','.runtime\pycache','.runtime\cache')) {
    New-Item -ItemType Directory -Path (Join-Path $projectRoot $folder) -Force | Out-Null
}
$env:MPLCONFIGDIR = Join-Path $projectRoot '.runtime\mpl'
$env:TEMP = Join-Path $projectRoot '.runtime\tmp'
$env:TMP = $env:TEMP
$env:TMPDIR = $env:TEMP
$env:PIP_CACHE_DIR = Join-Path $projectRoot '.runtime\pip-cache'
$env:PYTHONPYCACHEPREFIX = Join-Path $projectRoot '.runtime\pycache'
$env:XDG_CACHE_HOME = Join-Path $projectRoot '.runtime\cache'
$env:PYTHONDONTWRITEBYTECODE = '1'
$env:PYTHONUTF8 = '1'
if ($Mode -in @('Live','Demo')) {
    . (Join-Path $PSScriptRoot 'prepare_tk.ps1') -Python $pythonExe
}
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
