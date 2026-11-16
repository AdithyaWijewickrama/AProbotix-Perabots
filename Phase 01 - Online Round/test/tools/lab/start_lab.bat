@echo off
rem double click to start the sim lab, then open http://localhost:8765
cd /d "%~dp0"
start "" http://localhost:8765
python lab.py
pause
