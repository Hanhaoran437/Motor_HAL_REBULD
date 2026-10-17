# =============================================================================
#  cube-task.ps1 —— STM32Cube 工程的“可移植”编译/配置包装脚本
#
#  为什么需要它：
#    ST 官方示例里 tasks.json 直接写 cube.exe 的绝对路径，形如
#      C:/Users/<用户名>/.vscode/extensions/stmicroelectronics.stm32cube-ide-core-<版本>-win32-x64/...
#    用户名或扩展版本一变就失效。本脚本改为：
#      1) 先看 PATH 里有没有 cube（VS Code 装了 STM32Cube 扩展时任务里通常有）；
#      2) 再扫描 %USERPROFILE%\.vscode\extensions\ 下的 stm32cube-ide-core* 扩展目录；
#      3) 顺带处理跨机器复制工程留下的旧 CMake 缓存（自动删 build/<Preset> 重配）；
#      4) build 前若没有缓存，自动先 configure。
#
#  用法（tasks.json 已经这么调用了，一般不用手敲）：
#    powershell -NoProfile -ExecutionPolicy Bypass -File .vscode\cube-task.ps1 -Action build -Preset Debug
#    -Action  build | configure | clean
#    -Preset  Debug | Release
# =============================================================================
param(
    [ValidateSet('build', 'configure', 'clean')]
    [string]$Action = 'build',

    [ValidateSet('Debug', 'Release')]
    [string]$Preset = 'Debug',

    # 工作区根目录；默认取 .vscode 的上一级
    [string]$Root = ''
)

$ErrorActionPreference = 'Stop'

if (-not $Root) {
    $Root = Split-Path -Parent $PSScriptRoot
}
$Root = (Resolve-Path -LiteralPath $Root).Path
$BuildDir = Join-Path $Root ('build\' + $Preset)
$Cache = Join-Path $BuildDir 'CMakeCache.txt'

function Find-Cube {
    # 1) PATH 里的 cube（VS Code 任务/终端里通常由 STM32Cube 扩展注入）
    $cmd = Get-Command cube -ErrorAction SilentlyContinue
    if ($cmd -and $cmd.Source) { return $cmd.Source }

    # 2) 扫扩展目录，不依赖用户名和扩展版本号
    $extRoots = @(
        (Join-Path $env:USERPROFILE '.vscode\extensions'),
        (Join-Path $env:USERPROFILE '.vscode-insiders\extensions')
    )
    $hits = @()
    foreach ($extRoot in $extRoots) {
        if (-not (Test-Path -LiteralPath $extRoot)) { continue }
        Get-ChildItem -LiteralPath $extRoot -Directory -Filter '*stm32cube-ide-core*' -ErrorAction SilentlyContinue |
            ForEach-Object {
                $binRoot = Join-Path $_.FullName 'resources\binaries'
                if (Test-Path -LiteralPath $binRoot) {
                    $hits += Get-ChildItem -LiteralPath $binRoot -Recurse -Filter 'cube.exe' -ErrorAction SilentlyContinue
                }
            }
    }
    if ($hits.Count -gt 0) {
        return ($hits | Sort-Object FullName -Descending | Select-Object -First 1).FullName
    }
    return $null
}

$cube = Find-Cube
if (-not $cube) {
    Write-Host '找不到 cube.exe：请先安装 VS Code 扩展 “STM32Cube for VS Code”（stmicroelectronics.stm32cube-ide-core）。' -ForegroundColor Red
    exit 1
}

Write-Host "cube.exe    : $cube"
Write-Host "工作区      : $Root"
Write-Host "动作 / 预设 : $Action / $Preset"

if ($Action -eq 'clean') {
    if (Test-Path -LiteralPath $BuildDir) {
        Remove-Item -LiteralPath $BuildDir -Recurse -Force
        Write-Host "已删除 $BuildDir"
    }
    else {
        Write-Host "无需删除：$BuildDir 不存在"
    }
    exit 0
}

# ---- 跨机器/挪目录的旧缓存检测 ----
# CMakeCache.txt 里记着生成它的源码绝对路径，和当前工作区不一致时 CMake 会直接报错。
if (Test-Path -LiteralPath $Cache) {
    $line = Select-String -LiteralPath $Cache -Pattern '^CMAKE_HOME_DIRECTORY:INTERNAL=' -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($line) {
        $old = $line.Line.Substring($line.Line.IndexOf('=') + 1).Trim().Replace('/', '\').TrimEnd('\')
        $new = $Root.Replace('/', '\').TrimEnd('\')
        if ($old -and ($old -ne $new)) {
            Write-Host '检测到 build 缓存是另一台电脑/另一个目录生成的：' -ForegroundColor Yellow
            Write-Host "  旧源码目录: $old" -ForegroundColor Yellow
            Write-Host "  当前源码目录: $new" -ForegroundColor Yellow
            Write-Host '自动删除旧构建目录，重新配置…' -ForegroundColor Yellow
            Remove-Item -LiteralPath $BuildDir -Recurse -Force
        }
    }
}

$needConfigure = -not (Test-Path -LiteralPath $Cache)

if ($Action -eq 'configure') {
    $needConfigure = $true
}
elseif ($Action -eq 'build' -and $needConfigure) {
    Write-Host '还没有构建目录，先自动配置一次…'
}

if ($needConfigure) {
    Write-Host '>>> cube cmake --preset ' $Preset
    & $cube --cwd $Root cmake --preset $Preset
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

if ($Action -eq 'build') {
    Write-Host '>>> cube cmake --build --preset ' $Preset
    & $cube --cwd $Root cmake --build --preset $Preset
    exit $LASTEXITCODE
}

exit 0
