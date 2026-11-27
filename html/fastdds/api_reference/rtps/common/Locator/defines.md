<a id="macro-definitions-define"></a>

# Macro definitions (#define)

### LOCATOR_INVALID(loc)                                {loc.kind = [LOCATOR_KIND_INVALID](#Locator_8hpp_1a1c93e21e8adbaa8990ce44c1fd2a1c20); loc.port = [LOCATOR_PORT_INVALID](#Locator_8hpp_1ac88f3a8fef21ac106f79e085f7f15d51); \\<br/>                               [LOCATOR_ADDRESS_INVALID](#Locator_8hpp_1a03edab87fd5bd94be10c42174ca74bfe)<br/><br/>(loc.address); \\<br/><br/>}

Initialize locator with invalid values. 

### LOCATOR_KIND_INVALID -1

Invalid locator kind. 

### LOCATOR_ADDRESS_INVALID(a) {std::memset(a, 0x00, 16 \* sizeof(octet));}

Set locator IP address to 0. 

### LOCATOR_PORT_INVALID 0

Invalid locator port. 

### LOCATOR_KIND_RESERVED 0

Reserved locator kind. 

### LOCATOR_KIND_UDPv4 1

UDP over IPv4 locator kind. 

### LOCATOR_KIND_UDPv6 2

UDP over IPv6 locator kind. 

### LOCATOR_KIND_TCPv4 4

TCP over IPv4 kind. 

### LOCATOR_KIND_TCPv6 8

TCP over IPv6 locator kind. 

### LOCATOR_KIND_SHM 16 + FASTDDS_VERSION_MAJOR

Shared memory locator kind.
