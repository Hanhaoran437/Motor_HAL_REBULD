# =============================================================================
#  get-openocd.ps1 —— 把 xPack OpenOCD 放到 <工作区>/tools/openocd/
#
#  目的：无线调试器（CMSIS-DAP）需要的 GDB Server 是 OpenOCD，而 ST 官方渠道
#        不提供 OpenOCD，所以要么下载，要么从已有安装复制过来。
#
#  用法（在任意终端里，工作区根目录执行）：
#    powershell -NoProfile -ExecutionPolicy Bypass -File tools\get-openocd.ps1
#        默认下载 xpack-openocd-0.12.0-7-win32-x64.zip，github 直连不通时自动换镜像。
#
#    powershell ... -File tools\get-openocd.ps1 -Version 0.12.0-7 -Force
#        指定版本 / 覆盖已有文件。
#
#    powershell ... -File tools\get-openocd.ps1 -FromDir "C:\xpack-openocd-0.12.0-7"
#        不下载，直接从本机已有的 OpenOCD 目录复制（该目录下应有 bin\openocd.exe）。
#
#  说明：本脚本只往工作区的 tools\openocd 写文件，不动系统、不写注册表、不改 PATH。
# =============================================================================
[CmdletBinding()]
param(
    [string]$Version = '0.12.0-7',
    [string]$Dest = '',
    [string]$FromDir = '',
    [switch]$Force
)

$ErrorActionPreference = 'Stop'

if (-not $Dest) {
    # $PSScriptRoot = <工作区>\tools ，所以目标就是 <工作区>\tools\openocd
    $Dest = Join-Path $PSScriptRoot 'openocd'
}
$Dest = [System.IO.Path]::GetFullPath($Dest)

Write-Host "目标目录: $Dest"

$target = Join-Path $Dest 'bin\openocd.exe'
if ((Test-Path -LiteralPath $target) -and (-not $Force)) {
    Write-Host "已存在 $target（要覆盖请加 -Force）" -ForegroundColor Green
    exit 0
}

function Move-OpenOcdTree {
    param([string]$SourceDir)

    # 传入的目录里可能直接有 bin\openocd.exe，也可能是解压出来的一层子目录
    $src = $SourceDir
    if (-not (Test-Path -LiteralPath (Join-Path $src 'bin\openocd.exe'))) {
        $inner = Get-ChildItem -LiteralPath $src -Directory -ErrorAction SilentlyContinue |
            Where-Object { Test-Path -LiteralPath (Join-Path $_.FullName 'bin\openocd.exe') } |
            Select-Object -First 1
        if ($inner) { $src = $inner.FullName }
    }
    if (-not (Test-Path -LiteralPath (Join-Path $src 'bin\openocd.exe'))) {
        throw "在 $SourceDir 下找不到 bin\openocd.exe，目录结构不对。"
    }

    if (Test-Path -LiteralPath $Dest) {
        Remove-Item -LiteralPath $Dest -Recurse -Force
    }
    New-Item -ItemType Directory -Path $Dest -Force | Out-Null

    Get-ChildItem -LiteralPath $src -Force | ForEach-Object {
        Copy-Item -LiteralPath $_.FullName -Destination $Dest -Recurse -Force
    }
}

# ---------- 模式一：从本机已有安装复制 ----------
if ($FromDir) {
    Write-Host "从已有目录复制: $FromDir"
    Move-OpenOcdTree -SourceDir (Resolve-Path -LiteralPath $FromDir).Path
    Write-Host "完成: $target" -ForegroundColor Green
    exit 0
}

# ---------- 模式二：下载 ----------
$file = "xpack-openocd-$Version-win32-x64.zip"
$gh = "https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v$Version/$file"
$urls = @(
    $gh,
    "https://ghfast.top/$gh",
    "https://gh-proxy.com/$gh",
    "https://ghproxy.net/$gh",
    "https://github.moeyy.xyz/$gh"
)

try { [Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12 } catch { }

$work = Join-Path $env:TEMP ("openocd-dl-" + [Guid]::NewGuid().ToString('N').Substring(0, 8))
New-Item -ItemType Directory -Path $work -Force | Out-Null
$zip = Join-Path $work $file

$ok = $false
foreach ($u in $urls) {
    Write-Host "尝试下载: $u"
    try {
        Invoke-WebRequest -Uri $u -OutFile $zip -UseBasicParsing -TimeoutSec 600
        $size = (Get-Item -LiteralPath $zip).Length
        if ($size -lt 1MB) { throw "文件太小（$size 字节），不是正常安装包" }
        Write-Host ("下载成功，{0:N1} MB" -f ($size / 1MB)) -ForegroundColor Green
        $ok = $true
        break
    }
    catch {
        Write-Host "  失败: $($_.Exception.Message)" -ForegroundColor Yellow
    }
}

if (-not $ok) {
    Write-Host ''
    Write-Host '所有下载地址都失败。请手动下载后使用 -FromDir，或者直接手动解压：' -ForegroundColor Red
    Write-Host "  $gh" -ForegroundColor Red
    Write-Host "  解压后把 bin/ scripts/ share/ 放到: $Dest" -ForegroundColor Red
    exit 1
}

Write-Host '解压…'
Expand-Archive -LiteralPath $zip -DestinationPath $work -Force
Move-OpenOcdTree -SourceDir $work
Remove-Item -LiteralPath $work -Recurse -Force -ErrorAction SilentlyContinue

Write-Host "完成: $target" -ForegroundColor Green
Write-Host '现在可以直接在 VS Code 里按 F5 调试了。'
