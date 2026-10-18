@echo off
rem ===========================================================================
rem  build.cmd - double-click or run: tools\build.cmd
rem  Builds the firmware without the VS Code extension.
rem
rem    tools\build.cmd                 incremental Debug build
rem    tools\build.cmd -Clean          full rebuild
rem    tools\build.cmd -Preset Release Release build
rem ===========================================================================
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build.ps1" %*
