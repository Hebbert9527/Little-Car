param([string]$Gcc = 'D:/Program Files (x86)/Dev-Cpp/MinGW32/bin/gcc.exe')
$ErrorActionPreference = 'Stop'
$project = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$env:PATH = (Split-Path $Gcc) + ';' + $env:PATH
Push-Location $project
try {
    # SSE avoids old MinGW x87 excess precision changing boundary comparisons.
    $sources = @('tests/navigation/test_nav.c','tests/navigation/mock_port.c',
        'week4/Src/w4_common.c','week4/Src/w4_pose_single.c','week4/Src/w4_control.c',
        'week4/Src/w4_motor.c','week4/Src/w4_path.c','week4/Src/w4_bt_rx.c')
    & $Gcc -std=c11 -msse2 -mfpmath=sse -DW4_SINGLE_ENCODER -Wall -Wextra -Werror `
        -Iweek4/Inc -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 `
        @sources -lm -o tests/navigation/test_nav.exe
    if ($LASTEXITCODE -ne 0) { throw 'Navigation regression compilation failed' }
    & ./tests/navigation/test_nav.exe
    if ($LASTEXITCODE -ne 0) { throw 'Navigation regressions failed' }
} finally { Pop-Location }
