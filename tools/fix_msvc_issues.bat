@echo off
setlocal

REM Runs the MSVC fix-up script from the repo root.
REM Usage (from repo root):
REM   tools\fix_msvc_issues.bat

python "%~dp0fix_msvc_issues.py" --repo "%~dp0.."
exit /b %ERRORLEVEL%
