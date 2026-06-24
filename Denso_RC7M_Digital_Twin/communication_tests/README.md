# RC7M Communication Tests

These tests are ordered from least controller-dependent to most
controller-dependent.

## Test A: Ubuntu network reachability

Connect the Ubuntu PC and controller/gateway to an isolated network.

```bash
cd /home/airlab/Desktop/Robotic-Harvesting/Denso_RC7M_Digital_Twin
python3 communication_tests/ubuntu_network_probe.py 192.168.0.1
```

Replace `192.168.0.1` with the RC7M IP.

Test additional documented ports:

```bash
python3 communication_tests/ubuntu_network_probe.py 192.168.0.1 \
  --ports 5007,5008 --json
```

Port 5007 is the default in the official modern DENSO b-CAP ROS 2 tooling.
That does not prove RC7M uses or supports it.

Results:

- Ping passes, port closed: basic network works, b-CAP listener is not visible.
- Ping fails, port opens: ICMP may be blocked; TCP service is reachable.
- Both fail: check subnet, cable, firewall, controller Ethernet setup, and IP.
- Port opens: proceed to an official client/provider test; do not send motion.

## Test B: Windows-to-Ubuntu gateway transport

On Ubuntu:

```bash
python3 communication_tests/listen_for_gateway.py --port 15000
```

On Windows PowerShell:

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\Send-Rc7mTelemetryTest.ps1 -UbuntuIp 192.168.0.20
```

This tests only the new telemetry link, not RC7M.

## Test C: Windows ORiN read-only connection

Prerequisites:

- DENSO ORiN2 runtime/SDK installed
- valid license
- the RC7 provider installed and visible in CaoConfig
- exact provider name and option syntax from the RC7 documentation

Run from 32-bit or 64-bit PowerShell matching the installed ORiN COM runtime:

```powershell
Set-ExecutionPolicy -Scope Process Bypass

.\Test-Rc7mOrinReadOnly.ps1 `
  -ControllerIp 192.168.0.1 `
  -Provider "YOUR_RC7_PROVIDER_FROM_CAOCONFIG" `
  -Machine "localhost" `
  -Option "YOUR_PROVIDER_OPTION_STRING"
```

An option string sometimes resembles:

```text
conn=eth:192.168.0.1
```

This is only an example. Do not use it unless your installed provider manual
specifies it.

The script:

- creates `Cao.CaoEngine`
- opens workspace 0
- calls `AddController`
- enumerates robot names
- enumerates up to 30 controller variable names
- writes a JSON report

It does not:

- take arm ownership
- enable the servo
- start a task
- write variables
- reset alarms
- move the robot

## Test D: Official b-CAP ROS client

Only attempt this if Test A shows the documented b-CAP port and DENSO confirms
RC7M/provider compatibility.

The official DENSO ROS 2 repository offers a low-level `bcap_service`, but its
real-controller instructions explicitly target RC8/RC8A. Do not copy RC8
provider strings or motion examples to RC7M.

Safe initial operations are limited to:

1. start `bcap_service`
2. Controller Connect
3. get robot names
4. disconnect

Do not call `TakeArm`, `Motor`, `Move`, task start, variable put, or reset.

## What to send back for adapter development

Provide:

- network-probe output
- screenshot of CaoConfig providers
- ORiN test JSON report
- RC7M firmware version
- controller communication manual title/version
- whether robot and variable enumeration succeeded

With those results, the real read-only adapter can be implemented without
guessing provider names or variable paths.
