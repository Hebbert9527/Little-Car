param(
    [string]$Python = 'C:\Program Files\WindowsApps\OpenAI.CodexPrimaryRuntime.v26-1117-835-0_26.1117.835.0_x64__3k8sg7r9htsxt\dependencies\python\python.exe',
    [switch]$SkipInstall
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$workRoot = Split-Path -Parent $projectRoot
$runtimeRoot = Join-Path $projectRoot '.runtime'
foreach ($name in @('tmp','pip-cache','wheelhouse','mpl','pycache','cache')) {
    New-Item -ItemType Directory -Path (Join-Path $runtimeRoot $name) -Force | Out-Null
}
$taskTemp = Join-Path $runtimeRoot 'tmp'
$env:TEMP = $taskTemp
$env:TMP = $taskTemp
$env:TMPDIR = $taskTemp
$env:PIP_CACHE_DIR = Join-Path $runtimeRoot 'pip-cache'
$env:MPLCONFIGDIR = Join-Path $runtimeRoot 'mpl'
$env:PYTHONPYCACHEPREFIX = Join-Path $runtimeRoot 'pycache'
$env:XDG_CACHE_HOME = Join-Path $runtimeRoot 'cache'
$env:PYTHONDONTWRITEBYTECODE = '1'
$env:PYTHONUTF8 = '1'
$venvRoot = Join-Path $projectRoot '.venv'
$venvPython = Join-Path $venvRoot 'Scripts\python.exe'
if (-not (Test-Path -LiteralPath $venvPython)) {
    if (-not (Test-Path -LiteralPath $Python)) { throw 'Pass -Python with an existing Python 3.12 executable.' }
    & $Python -B -m venv --system-site-packages $venvRoot
    if ($LASTEXITCODE -ne 0) { throw 'Virtual environment creation failed' }
}
$cfgPath = Join-Path $venvRoot 'pyvenv.cfg'
$cfg = Get-Content -LiteralPath $cfgPath -Raw
$cfg.Replace('include-system-site-packages = false','include-system-site-packages = true') | Set-Content -LiteralPath $cfgPath -Encoding utf8
& $venvPython -B -c 'import json,os,tempfile; print(json.dumps({k:os.environ.get(k) for k in ("TEMP","TMP","TMPDIR","PIP_CACHE_DIR","MPLCONFIGDIR")},indent=2)); print("Python tempfile:",tempfile.gettempdir())'
if ($LASTEXITCODE -ne 0) { throw 'Python temporary directory verification failed' }
if ($SkipInstall) { return }
$localWheels = @(Get-ChildItem -LiteralPath (Join-Path $runtimeRoot 'wheelhouse') -Filter '*.whl' -File | Where-Object { $_.Name -match '^(matplotlib|pyserial|bleak|contourpy|cycler|fonttools|kiwisolver)-' } | ForEach-Object { $_.FullName })
& $venvPython -B -m pip install --disable-pip-version-check --timeout 15 --retries 2 --progress-bar off --cache-dir $env:PIP_CACHE_DIR --find-links (Join-Path $runtimeRoot 'wheelhouse') @localWheels -r (Join-Path $projectRoot 'pc\requirements.txt') 2>&1 | Tee-Object -FilePath (Join-Path $workRoot 'logs\pc_dependencies_D_only_20261118.txt')
if ($LASTEXITCODE -ne 0) { throw 'PC dependency installation failed; see D-only log' }
