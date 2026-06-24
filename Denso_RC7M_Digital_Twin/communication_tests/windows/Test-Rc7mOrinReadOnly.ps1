param(
    [Parameter(Mandatory = $true)]
    [string]$ControllerIp,

    [string]$ControllerName = "RC7M_Test",

    # Obtain the exact provider string from CaoConfig or the installed RC7
    # provider documentation. Common examples from other generations must not
    # be assumed correct for this controller.
    [Parameter(Mandatory = $true)]
    [string]$Provider,

    [string]$Machine = "localhost",

    # Obtain the exact option syntax from the provider documentation.
    # Example only: "conn=eth:192.168.0.1"
    [Parameter(Mandatory = $true)]
    [string]$Option,

    [string]$OutputJson = ".\rc7m_readonly_result.json"
)

$ErrorActionPreference = "Stop"

function Add-Result {
    param([string]$Step, [bool]$Success, [string]$Detail)
    $script:Results += [PSCustomObject]@{
        step = $Step
        success = $Success
        detail = $Detail
        time_utc = [DateTime]::UtcNow.ToString("o")
    }
}

$Results = @()
$Controller = $null
$Engine = $null

Write-Host "READ-ONLY RC7M ORiN TEST"
Write-Host "No servo, task start, variable write, reset, or motion calls are used."
Write-Host ""

try {
    $ping = Test-Connection -ComputerName $ControllerIp -Count 1 -Quiet
    Add-Result "ping" $ping "Controller IP: $ControllerIp"
    if (-not $ping) {
        throw "Controller did not answer ping. Some controllers block ICMP; continue only after checking network settings."
    }

    try {
        $Engine = New-Object -ComObject "Cao.CaoEngine"
        Add-Result "create_cao_engine" $true "Cao.CaoEngine COM object created."
    }
    catch {
        Add-Result "create_cao_engine" $false $_.Exception.Message
        throw "ORiN2 runtime/SDK is missing, unlicensed, or not registered for this Windows architecture."
    }

    $Workspace = $Engine.Workspaces.Item(0)
    Add-Result "get_workspace" ($null -ne $Workspace) "Workspace index 0."

    # AddController establishes a provider connection. It does not request arm
    # ownership, enable the motor, execute a task, or write a variable.
    $Controller = $Workspace.AddController(
        $ControllerName,
        $Provider,
        $Machine,
        $Option
    )
    Add-Result "add_controller" ($null -ne $Controller) "Provider=$Provider; Machine=$Machine; Option=$Option"

    $RobotNames = @()
    try {
        foreach ($Robot in $Controller.Robots) {
            $RobotNames += [string]$Robot.Name
        }
        Add-Result "enumerate_robots" $true (($RobotNames -join ", "))
    }
    catch {
        Add-Result "enumerate_robots" $false $_.Exception.Message
    }

    $VariableNames = @()
    try {
        $Count = 0
        foreach ($Variable in $Controller.Variables) {
            if ($Count -ge 30) { break }
            $VariableNames += [string]$Variable.Name
            $Count++
        }
        Add-Result "enumerate_first_variables" $true (($VariableNames -join ", "))
    }
    catch {
        Add-Result "enumerate_first_variables" $false $_.Exception.Message
    }

    Write-Host "Connection test completed."
    Write-Host "Robots: $($RobotNames -join ', ')"
}
catch {
    Write-Error $_.Exception.Message
    Add-Result "fatal" $false $_.Exception.Message
}
finally {
    # Release COM objects without invoking any controller command.
    if ($null -ne $Controller) {
        try { [void][Runtime.InteropServices.Marshal]::FinalReleaseComObject($Controller) } catch {}
    }
    if ($null -ne $Engine) {
        try { [void][Runtime.InteropServices.Marshal]::FinalReleaseComObject($Engine) } catch {}
    }

    $Report = [PSCustomObject]@{
        test = "denso_rc7m_orin_read_only"
        controller_ip = $ControllerIp
        provider = $Provider
        machine = $Machine
        option = $Option
        results = $Results
        warning = "Connection/read test only. No motion authorization is implied."
    }
    $Report | ConvertTo-Json -Depth 6 | Set-Content -Path $OutputJson -Encoding UTF8
    Write-Host "Report saved: $OutputJson"
}
