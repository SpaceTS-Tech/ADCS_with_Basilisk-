# Rebuild the double-click .exe launchers from Launcher.cs.
#
#     powershell -ExecutionPolicy Bypass -File tools\launcher\build_launchers.ps1
#
# Uses the C# compiler that ships with the .NET Framework, so nothing has to be
# installed.  Re-run this after editing Launcher.cs or renaming an entry point.
#
# Pass -Only with .exe names to rebuild just those, e.g.
#
#     build_launchers.ps1 -Only Run_Connector.exe

param([string[]] $Only)

$ErrorActionPreference = 'Stop'

$launcherDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$root        = Split-Path -Parent (Split-Path -Parent $launcherDir)
$template    = Join-Path $launcherDir 'Launcher.cs'
# Windowed apps (Gui = $true) use this instead: pythonw, no console window.
$guiTemplate = Join-Path $launcherDir 'GuiLauncher.cs'

$csc = Join-Path $env:WINDIR 'Microsoft.NET\Framework64\v4.0.30319\csc.exe'
if (-not (Test-Path $csc)) {
    $csc = Join-Path $env:WINDIR 'Microsoft.NET\Framework\v4.0.30319\csc.exe'
}
if (-not (Test-Path $csc)) {
    throw "C# compiler not found. Enable the .NET Framework 4 feature, or build the launchers on a machine that has it."
}

# Start-up menu offered when the .exe is double-clicked with no arguments.
# Entries are 'label|description|arguments', joined by ';;'.  The first entry is
# the default; arguments of '?' prompt the user to type their own.  Leave a
# target's Menu empty to run its script straight away with no questions.
$simulatorMenu = @(
    'File mode|runs for a fixed time, then writes the plots and the Vizard recording into output\|',
    'Live mode|streams to Vizard for interactive 3-D; the keyboard sets the mission command|--mode live',
    'File mode, short run|the same as [1] but 500 s instead of 5000 s, for a quick check|--seconds 500',
    'No flight software|live mode with no ADCS software: no control torque or force|--mode live --no-flight-software',
    'Custom|type your own arguments|?'
) -join ';;'

# folder -> the script its .exe runs, the .exe name, the window title, the
# arguments used when there is no menu, and the menu itself
$targets = @(
    @{ Folder = 'simulator';        Script = 'scenarioADCSSoftwareRW.py';        Exe = 'Run_Simulator.exe';         Title = 'ADCS Simulator';        Args = ''; Menu = $simulatorMenu },
    @{ Folder = 'regression_tests'; Script = 'test_mission_modes.py';            Exe = 'Run_Regression_Tests.exe';  Title = 'ADCS Regression Tests'; Args = ''; Menu = '' },
    @{ Folder = 'test_report';      Script = 'run_test_cases.py';                Exe = 'Run_Test_Cases.exe';        Title = 'ADCS Test Case Report'; Args = ''; Menu = '' },
    @{ Folder = 'connector';        Script = 'connector_app.py';                 Exe = 'Run_Connector.exe';         Title = 'AOCS Connector';        Args = ''; Menu = ''; Gui = $true },
    @{ Folder = 'legacy';           Script = 'scenarioADCSSoftwareRW_working.py'; Exe = 'Run_Legacy_Scenario.exe';  Title = 'ADCS Legacy Scenario';  Args = ''; Menu = '' }
)

# Values land inside C# string literals, so a backslash in a path or a menu
# description would otherwise be read as an escape sequence.
function ConvertTo-CSharpLiteral([string] $text) {
    if ([string]::IsNullOrEmpty($text)) { return '' }
    return $text.Replace('\', '\\').Replace('"', '\"')
}

$source    = Get-Content $template -Raw
$guiSource = Get-Content $guiTemplate -Raw
$temp   = Join-Path $env:TEMP ("adcs_launcher_" + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $temp -Force | Out-Null

try {
    foreach ($t in $targets) {
        if ($Only -and ($Only -notcontains $t.Exe)) { continue }
        $outDir = Join-Path $root $t.Folder
        if (-not (Test-Path (Join-Path $outDir $t.Script))) {
            Write-Warning "skipped $($t.Exe): $($t.Folder)\$($t.Script) not found"
            continue
        }

        $isGui = [bool]$t.Gui
        $generated = $(if ($isGui) { $guiSource } else { $source }).
            Replace('@@SCRIPT@@',      (ConvertTo-CSharpLiteral $t.Script)).
            Replace('@@TITLE@@',       (ConvertTo-CSharpLiteral $t.Title)).
            Replace('@@DEFAULTARGS@@', (ConvertTo-CSharpLiteral $t.Args)).
            Replace('@@MENU@@',        (ConvertTo-CSharpLiteral $t.Menu))

        $csFile = Join-Path $temp ($t.Folder + '.cs')
        Set-Content -Path $csFile -Value $generated -Encoding UTF8

        $exePath = Join-Path $outDir $t.Exe
        if ($isGui) {
            & $csc /nologo /target:winexe /optimize+ /warn:4 /reference:System.Windows.Forms.dll /out:"$exePath" "$csFile"
        } else {
            & $csc /nologo /target:exe /optimize+ /warn:4 /out:"$exePath" "$csFile"
        }
        if ($LASTEXITCODE -ne 0) { throw "compilation failed for $($t.Exe)" }

        Write-Host ("built  {0,-26} -> {1}\{2}" -f $t.Exe, $t.Folder, $t.Script)
    }
}
finally {
    Remove-Item $temp -Recurse -Force -ErrorAction SilentlyContinue
}

Write-Host ''
Write-Host 'Done. Double-click any of the .exe files to run its folder''s entry point.'
