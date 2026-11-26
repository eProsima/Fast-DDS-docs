<a id="portparameters"></a>

# PortParameters

### class PortParameters

Class [PortParameters](#classeprosima_1_1fastdds_1_1rtps_1_1PortParameters), to define the port parameters and gains related with the RTPS protocol. 

### Public Functions

### inline std::uint32_t getMulticastPort(std::uint32_t domainId) const

Get a multicast port based on the domain ID.

* **Parameters:**
  **domainId** – Domain ID. 
* **Returns:**
  Multicast port 

### inline std::uint32_t getUnicastPort(std::uint32_t domainId, std::uint32_t RTPSParticipantID) const

Get a unicast port based on the domain ID and the participant ID.

* **Parameters:**
  * **domainId** – Domain ID. 
  * **RTPSParticipantID** – Participant ID. 
* **Returns:**
  Unicast port 

### inline std::uint16_t get_discovery_server_port(std::uint32_t domainId) const

Get a discovery server port based on the domain ID.

* **Parameters:**
  **domainId** – Domain ID. 
* **Returns:**
  Discovery server port 

### Public Members

### std::uint16_t portBase

PortBase, default value 7400. 

### std::uint16_t domainIDGain

DomainID gain, default value 250. 

### std::uint16_t participantIDGain

ParticipantID gain, default value 2. 

### std::uint16_t offsetd0

Offset d0, default value 0. 

### std::uint16_t offsetd1

Offset d1, default value 10. 

### std::uint16_t offsetd2

Offset d2, default value 1. 

### std::uint16_t offsetd3

Offset d3, default value 11. 

### std::uint16_t offsetd4

Offset d4, default value 2.
