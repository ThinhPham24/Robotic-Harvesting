# RC7M Capability Audit

Complete this checklist before writing the real Windows adapter.

## Controller identity

- Full controller label and serial number
- RC7M firmware/software version
- Teach pendant model
- Robot model confirmation: VS-6556E
- Installed expansion boards
- Existing PLC and network topology

## Communication

- Ethernet port physically present
- Controller IP can be configured
- Controller responds to ping
- ORiN/CAO provider name and version
- ORiN2 SDK license available
- b-CAP listener/server available
- Supported TCP/UDP socket functions in PACScript
- Supported fieldbus: EtherNet/IP, CC-Link, DeviceNet, PROFIBUS, other
- Accessible controller variables
- Accessible current joint position
- Accessible alarm history
- Accessible servo/mode/E-stop state
- Accessible motor current, torque, temperature, or cycle counters

## Motion authority

- Can an external client start an existing PACScript task?
- Can an external client write bounded position variables?
- Is there an execution token or remote-mode interlock?
- Is external interpolation supported?
- What command rate and acknowledgement behavior are documented?
- How is an external command cancelled?

## Required evidence

Collect:

- controller manuals
- ORiN provider documentation
- screenshots of installed licenses/options
- a list of existing PACScript programs
- network settings
- one read-only PC test showing joint/status values

Do not proceed to command development based only on the controller family name.
