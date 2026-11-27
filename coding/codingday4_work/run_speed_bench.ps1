param(
    [ValidateRange(60,110)][int]$TargetRpm = 90,
    [string]$BuildDir = '.\firmware_source\build\motor_speed_loop_v7',
    [string]$Manifest = '.\logs\verified_motor_speed_loop_v7_20261119_162025.json',
    [float]$Kp = -1,
    [float]$Ki = -1
)
$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
$cli = python -B -c 'from read_current_board import CLI; print(CLI)'
if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $cli -PathType Leaf)) { throw 'STM32CubeProgrammer CLI not found' }
$build = $BuildDir
$stamp = [DateTime]::UtcNow.AddHours(8).ToString('yyyyMMdd_HHmmss_fff')
$prefix = "speed_pre_${TargetRpm}_$stamp"
python -B read_motor_pulse.py --label $prefix --verify-flash --build-dir $build --manifest $manifest | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'Pre-test identity verification failed' }
$identity = Get-ChildItem -LiteralPath '.\logs' -Filter "$prefix*.json" | Select-Object -First 1
$before = Get-Content -Raw -LiteralPath $identity.FullName | ConvertFrom-Json
if ($before.diagnostic.request -ne 0 -or $before.diagnostic.state -notin @(0,2)) { throw 'Board is not idle' }
if ($Kp -ne -1 -or $Ki -ne -1) {
    if ([float]::IsNaN($Kp) -or [float]::IsNaN($Ki) -or
        $Kp -lt 0 -or $Kp -gt 0.01 -or $Ki -lt 0 -or $Ki -gt 0.05) { throw 'Invalid PI gains' }
    $gainAddress = python -B -c 'from pathlib import Path; import sys,read_motor_pulse as m; print(hex(m.symbol_addresses(Path(sys.argv[1]),True)["w4_wheel_speed_gains"]))' $build
    if ($LASTEXITCODE -ne 0) { throw 'Cannot identify PI gain address' }
    $kpWord = '0x{0:X8}' -f [BitConverter]::ToUInt32([BitConverter]::GetBytes($Kp),0)
    $kiWord = '0x{0:X8}' -f [BitConverter]::ToUInt32([BitConverter]::GetBytes($Ki),0)
    $tuning = & $cli -c port=SWD sn=0674FF545155657067032031 mode=HOTPLUG -w32 $gainAddress $kpWord $kiWord 2>&1
    $tuningCode = $LASTEXITCODE
    $tuning | Out-String -Width 4096 | Set-Content -LiteralPath ".\logs\speed_gains_$stamp.txt" -Encoding UTF8
    if ($tuningCode -ne 0) { throw 'Could not verify PI gain write; no pulse sent' }
}
$request = '0x{0:X8}' -f (0x53500000 -bor $TargetRpm)
# The firmware immediately consumes the request. Read the completed counter
# after any CLI write error, since retrying could trigger a second test.
$report = & $cli -c port=SWD sn=0674FF545155657067032031 mode=HOTPLUG -nv -w32 $before.diagnostic_address $request 2>&1
$writeCode = $LASTEXITCODE
$report | Out-String -Width 4096 | Set-Content -LiteralPath ".\logs\speed_request_${TargetRpm}_$stamp.txt" -Encoding UTF8
Write-Output "Single speed request sent. CLI exit $writeCode. Reading result without retry."
Start-Sleep -Milliseconds 5250
$prefix = "speed_post_${TargetRpm}_$stamp"
python -B read_motor_pulse.py --label $prefix --identity $identity.FullName --build-dir $build --manifest $manifest --speed-trace
if ($LASTEXITCODE -ne 0) { throw 'Test ended with a diagnostic error; inspect saved trace' }
$post = Get-ChildItem -LiteralPath '.\logs' -Filter "$prefix*.json" | Where-Object { $_.Name -notlike '*_speed.json' } | Select-Object -First 1
$after = Get-Content -Raw -LiteralPath $post.FullName | ConvertFrom-Json
$diag = $after.diagnostic
if ($diag.completed -ne $before.diagnostic.completed + 1 -or $diag.state -ne 2 -or
    $diag.last_side -ne 3 -or [Math]::Abs(($diag.stop_ms - $diag.start_ms) - 5000) -gt 2 -or
    $after.speed_summary.target_rpm -ne $TargetRpm) { throw 'Recorded test differs from the requested speed pulse' }
