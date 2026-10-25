param([string]$seed, [string]$mode = "")
$h = $PSScriptRoot
$proj = (Resolve-Path "$h\..\..").Path
$out = "$h\runs\r$seed$mode"
if (Test-Path $out) { Remove-Item -Recurse -Force $out }
python "$h\gen.py" $proj $out $seed $mode
$env:WEBOTS_HOME = "D:\Programs\Webots"
$env:PATH = "D:\Programs\Webots\msys64\mingw64\bin;D:\Programs\Webots\msys64\mingw64\bin\cpp;D:\Programs\Webots\msys64\usr\bin;" + $env:PATH
$p = Start-Process -FilePath "D:\Programs\Webots\msys64\mingw64\bin\webots-bin.exe" -ArgumentList "--batch","--mode=fast","--no-rendering","--minimize","`"$out\worlds\test.wbt`"" -PassThru
if (-not $p.WaitForExit(600000)) { $p.Kill(); "TIMEOUT" }
Get-Content "$out\controllers\main_controller\ctrl_log.txt" -ErrorAction SilentlyContinue | Select-Object -Last 40
Get-Content "$out\controllers\judge\judge_log.txt" -ErrorAction SilentlyContinue
