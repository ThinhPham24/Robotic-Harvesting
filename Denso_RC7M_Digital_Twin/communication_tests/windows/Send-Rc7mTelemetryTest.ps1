param(
    [Parameter(Mandatory = $true)]
    [string]$UbuntuIp,
    [int]$Port = 15000,
    [int]$Count = 100,
    [double]$RateHz = 20.0
)

$Udp = New-Object System.Net.Sockets.UdpClient
$Endpoint = New-Object System.Net.IPEndPoint ([System.Net.IPAddress]::Parse($UbuntuIp)), $Port
$PeriodMs = [Math]::Max(1, [int](1000.0 / $RateHz))

try {
    for ($Sequence = 0; $Sequence -lt $Count; $Sequence++) {
        $NowNs = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() * 1000000
        $Packet = @{
            protocol = "denso_rc7m.telemetry.v1"
            sequence = $Sequence
            source_time_unix_ns = $NowNs
            controller = "WINDOWS-GATEWAY-TEST"
            robot = "VS-6556E"
            joint_names = @("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
            joint_position_deg = @(0.0, -20.0, 45.0, 0.0, 30.0, 0.0)
            joint_velocity_deg_s = @(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            mode = "MANUAL"
            servo_on = $false
            emergency_stop = $false
            protective_stop = $false
            alarm_code = 0
            alarm_text = ""
            native_task = "GATEWAY_NETWORK_TEST"
            cycle_count = 0
        }
        $Json = $Packet | ConvertTo-Json -Compress
        $Bytes = [Text.Encoding]::UTF8.GetBytes($Json)
        [void]$Udp.Send($Bytes, $Bytes.Length, $Endpoint)
        Start-Sleep -Milliseconds $PeriodMs
    }
}
finally {
    $Udp.Close()
}

Write-Host "Sent $Count telemetry test packets to ${UbuntuIp}:$Port"
