<a id="api-transport-receiver-interface"></a>

<a id="transportreceiverinterface"></a>

# TransportReceiverInterface

### class TransportReceiverInterface

Interface against which to implement a data receiver, decoupled from transport internals. 

### Public Functions

### virtual ~TransportReceiverInterface() = default

Destructor. 

### virtual void OnDataReceived(const fastdds::rtps::[octet](../../rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE) \*data, const std::uint32_t size, const Locator &local_locator, const Locator &remote_locator) = 0

Method to be called by the transport when receiving data. 

* **Parameters:**
  * **data** – Pointer to the received data. 
  * **size** – Number of bytes received. 
  * **local_locator** – Locator identifying the local endpoint. 
  * **remote_locator** – Locator identifying the remote endpoint.
