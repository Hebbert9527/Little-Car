param([Parameter(Mandatory=$true)][string]$Python)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$runtimeRoot = Join-Path $projectRoot '.runtime'
$localTcl = Join-Path $runtimeRoot 'tcl'
$sourceInfo = & $Python -B -c 'import json,sys,_tkinter; print(json.dumps(dict(base=sys.base_prefix,tcl=_tkinter.TCL_VERSION,tk=_tkinter.TK_VERSION)))'
if ($LASTEXITCODE -ne 0) { throw 'Cannot identify the Python Tcl/Tk runtime.' }
$info = $sourceInfo | ConvertFrom-Json
$sourceTcl = Join-Path $info.base 'tcl'
$initRelative = 'tcl' + $info.tcl + '\init.tcl'
$tkRelative = 'tk' + $info.tk + '\tk.tcl'
$marker = Join-Path $runtimeRoot 'tcl-source.json'
$matchingSource = $false
if (Test-Path -LiteralPath $marker) {
    $previous = Get-Content -LiteralPath $marker -Raw | ConvertFrom-Json
    $matchingSource = ($previous.base -eq $info.base -and $previous.tcl -eq $info.tcl -and $previous.tk -eq $info.tk)
}
if (-not $matchingSource -or -not (Test-Path -LiteralPath (Join-Path $localTcl $initRelative)) -or -not (Test-Path -LiteralPath (Join-Path $localTcl $tkRelative))) {
    if (-not (Test-Path -LiteralPath (Join-Path $sourceTcl $initRelative)) -or -not (Test-Path -LiteralPath (Join-Path $sourceTcl $tkRelative))) {
        throw "Matching Tcl/Tk scripts are absent from the selected Python runtime: $sourceTcl"
    }
    New-Item -ItemType Directory -Path $localTcl -Force | Out-Null
    foreach ($item in Get-ChildItem -LiteralPath $sourceTcl -Force) {
        Copy-Item -LiteralPath $item.FullName -Destination $localTcl -Recurse -Force
    }
    $info | ConvertTo-Json | Set-Content -LiteralPath $marker -Encoding utf8
}
$env:TCL_LIBRARY = Join-Path $localTcl ('tcl' + $info.tcl)
$env:TK_LIBRARY = Join-Path $localTcl ('tk' + $info.tk)