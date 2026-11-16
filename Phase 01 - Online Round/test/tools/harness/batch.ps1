param([int[]]$seeds, [string]$mode = "")
$h = $PSScriptRoot
$env:JLIMIT="400"; $env:JEVERY="1000000"
foreach ($s in $seeds) {
  & "$h\run.ps1" $s $mode | Out-Null
  $out = "$h\runs\r$s$mode"
  $truth = Get-Content "$out\truth.json" | ConvertFrom-Json
  $c = Get-Content "$out\controllers\main_controller\ctrl_log.txt" -ErrorAction SilentlyContinue | Where-Object { $_ -notmatch "^EST" }
  $maj = ($c | Select-String "explored\." | Select-Object -Last 1)
  $j = (Get-Content "$out\controllers\judge\judge_log.txt" -ErrorAction SilentlyContinue | Select-String "JUDGE mode") -replace "JUDGE ",""
  $want = if ($truth.blue -gt $truth.green) { "BLUE" } else { "GREEN" }
  $ok = if ("$maj" -match "-> $want") { "OK " } else { "BAD" }
  "seed=$s$mode truth b=$($truth.blue) g=$($truth.green) [$ok] | $maj | $j"
  $c | Select-String "wiggle|no path|Error|Traceback" | Select-Object -First 3 | ForEach-Object { "      ! $_" }
}
