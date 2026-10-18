# ============================================================================
#  flash.ps1 - flash from the command line (no VS Code extension, ~2-4 s)
#
#  Usage:
#     .\tools\flash.ps1                        # flash build\Debug\Motor_HAL_REBULD.elf
#     .\tools\flash.ps1 -Elf other\app.elf     # flash a specific file
#     .\tools\flash.ps1 -SpeedKHz 2000         # wired CMSIS-DAP can go faster
#     .\tools\flash.ps1 -Probe stlink          # use an ST-LINK instead
#     .\tools\flash.ps1 -NoReset               # program + verify, stay halted
#
#  Requires: debug probe connected, target board powered.
#  NOTE: keep this file ASCII-only (Windows PowerShell 5.1 reads .ps1 as the
#        ANSI code page, non-ASCII text breaks parsing).
# ============================================================================
param(
    [string]$Elf = '',
    [int]$SpeedKHz = 1000,
    [ValidateSet('cmsis-dap', 'stlink')]
    [string]$Probe = 'cmsis-dap',
    [string]$OpenOcd = '',
    [switch]$NoReset
)

$ErrorActionPreference = 'Stop'
$Root = Split-Path -Parent $PSScriptRoot          # tools\ -> project root

if (-not $Elf) { $Elf = Join-Path $Root 'build\Debug\Motor_HAL_REBULD.elf' }
if (-not (Test-Path $Elf)) { throw "Firmware not found: $Elf (run tools\build.cmd first)" }
$Elf = (Resolve-Path $Elf).Path

if (-not $OpenOcd) {
    $OpenOcd = Join-Path $Root 'tools\openocd\bin\openocd.exe'
    if (-not (Test-Path $OpenOcd)) {
        $cmd = Get-Command 'openocd.exe' -ErrorAction SilentlyContinue
        if (-not $cmd) { throw "openocd.exe not found - put it in tools\openocd\bin\ or pass -OpenOcd." }
        $OpenOcd = $cmd.Source
    }
}

$iface = if ($Probe -eq 'stlink') { 'interface/stlink.cfg' } else { 'interface/cmsis-dap.cfg' }
$tail  = if ($NoReset) { 'verify' } else { 'verify reset exit' }

# {path} braces = let Tcl treat the path (with spaces/CJK) as one token
$argList = @(
    '-f', $iface,
    '-c', "adapter speed $SpeedKHz",
    '-f', 'target/stm32f4x.cfg',
    '-c', "program {$Elf} $tail"
)

Write-Host "Firmware: $Elf" -ForegroundColor Cyan
Write-Host "Probe   : $Probe @ ${SpeedKHz} kHz" -ForegroundColor Cyan

& $OpenOcd @argList
if ($LASTEXITCODE -ne 0) { throw "flash failed (exit $LASTEXITCODE)" }

if (-not $NoReset) { Write-Host 'OK - programmed, verified, reset and running' -ForegroundColor Green }
