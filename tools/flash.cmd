@echo off
rem ===========================================================================
rem  flash.cmd - double-click or run: tools\flash.cmd
rem  Programs build\Debug\Motor_HAL_REBULD.elf through OpenOCD (2-4 s).
rem
rem    tools\flash.cmd                 CMSIS-DAP probe, 1000 kHz
rem    tools\flash.cmd -SpeedKHz 2000  faster clock (wired probe)
rem    tools\flash.cmd -Probe stlink   use an ST-LINK instead
rem ===========================================================================
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash.ps1" %*
