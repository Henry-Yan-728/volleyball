param(
    [string]$ElfPath = "$PSScriptRoot\\..\\build\\Debug\\volleyball2_H7.elf",
    [int]$AdapterSpeed = 1000,
    [string]$ProbeSerial = ""
)

$ErrorActionPreference = "Stop"

function Resolve-OpenOcdExe {
    $candidates = @()

    if ($env:OPENOCD_EXE) {
        $candidates += $env:OPENOCD_EXE
    }

    $cmd = Get-Command openocd -ErrorAction SilentlyContinue
    if ($cmd) {
        $candidates += $cmd.Source
    }

    $candidates += @(
        "D:\\robocon\\DevEnv\\DevEnv\\openocd-v0.12.0-i686-w64-mingw32\\bin\\openocd.exe",
        "F:\\STM32\\STM32CubeCLT_1.19.0\\OpenOCD\\bin\\openocd.exe",
        "C:\\ST\\STM32CubeCLT\\OpenOCD\\bin\\openocd.exe"
    )

    foreach ($candidate in $candidates) {
        if ($candidate -and (Test-Path $candidate)) {
            return (Resolve-Path $candidate).Path
        }
    }

    throw "openocd.exe not found. Set OPENOCD_EXE or install OpenOCD."
}

function Resolve-OpenOcdScripts([string]$openocdExe) {
    $binDir = Split-Path -Parent $openocdExe
    $rootDir = Split-Path -Parent $binDir

    $candidates = @(
        $env:OPENOCD_SCRIPTS,
        (Join-Path $rootDir "share\\openocd\\scripts"),
        (Join-Path $binDir "..\\..\\share\\openocd\\scripts")
    )

    foreach ($candidate in $candidates) {
        if ($candidate -and (Test-Path $candidate)) {
            return (Resolve-Path $candidate).Path
        }
    }

    throw "OpenOCD scripts path not found. Set OPENOCD_SCRIPTS."
}

$resolvedElf = (Resolve-Path $ElfPath).Path
$openocdExe = Resolve-OpenOcdExe
$openocdScripts = Resolve-OpenOcdScripts -openocdExe $openocdExe
$elfForOpenOcd = $resolvedElf -replace "\\", "/"

$openocdArgs = @(
    "-s", $openocdScripts,
    "-f", "interface/cmsis-dap.cfg",
    "-f", "target/stm32h7x.cfg",
    "-c", "cmsis_dap_backend hid",
    "-c", "transport select swd",
    "-c", "adapter speed $AdapterSpeed"
)

if ($ProbeSerial) {
    $openocdArgs += @("-c", "cmsis_dap_serial $ProbeSerial")
}

$openocdArgs += @("-c", "program `"$elfForOpenOcd`" verify reset exit")

Write-Host "[OpenOCD] EXE     : $openocdExe"
Write-Host "[OpenOCD] Scripts : $openocdScripts"
Write-Host "[OpenOCD] ELF     : $resolvedElf"

& $openocdExe @openocdArgs
exit $LASTEXITCODE

