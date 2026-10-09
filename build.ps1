param([ValidateSet('release','debug')][string]$Configuration='release', [switch]$Package)
$ErrorActionPreference='Stop'
$projectDir=$PSScriptRoot
$envFile=Join-Path $projectDir 'config/environment.ini'
if (!(Test-Path -LiteralPath $envFile)) { throw 'Create config/environment.ini from defaults first.' }
$settings=@{}
Get-Content -LiteralPath $envFile | ForEach-Object { if ($_ -match '^([^;#=\[]+)=(.*)$') { $settings[$Matches[1].Trim()]=$Matches[2].Trim() } }
$vsRoot=$settings['VS_ROOT']
if (!$vsRoot) {
  $vswhere=Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio/Installer/vswhere.exe'
  if (Test-Path -LiteralPath $vswhere) { $vsRoot=& $vswhere -latest -products '*' -property installationPath }
}
$devCmd=Join-Path $vsRoot 'Common7/Tools/VsDevCmd.bat'
if (!(Test-Path -LiteralPath $devCmd)) { throw 'VS_ROOT must point to Visual Studio with C++ desktop tools.' }
# Import compiler environment; this command performs no filesystem mutation.
$envRows=& cmd.exe /d /s /c "`"`"$devCmd`" -arch=x64 -host_arch=x64 >nul && set`""
if ($LASTEXITCODE -ne 0) { throw 'MSVC environment initialization failed.' }
foreach ($row in $envRows) { if ($row -match '^([^=]+)=(.*)$') { [Environment]::SetEnvironmentVariable($Matches[1], $Matches[2], 'Process') } }
$env:VSLANG='1033'
$cmake=Join-Path $vsRoot 'Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe'
$ctest=Join-Path (Split-Path $cmake) 'ctest.exe'
Push-Location $projectDir
try {
  & $cmake --preset $Configuration
  if ($LASTEXITCODE -ne 0) { throw 'Configure failed' }
  & $cmake --build --preset $Configuration
  if ($LASTEXITCODE -ne 0) { throw 'Build failed' }
  & $ctest --preset $Configuration
  if ($LASTEXITCODE -ne 0) { throw 'Tests failed' }
  if ($Package) {
    & $cmake --install "out/build/$Configuration" --prefix 'out/package/MLLM'
    if ($LASTEXITCODE -ne 0) { throw 'Install failed' }
  }
} finally { Pop-Location }

