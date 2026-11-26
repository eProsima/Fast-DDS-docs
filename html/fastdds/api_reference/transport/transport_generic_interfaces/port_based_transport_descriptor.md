<a id="api-port-based-transport-descriptor"></a>

<a id="portbasedtransportdescriptor"></a>

# PortBasedTransportDescriptor

### class PortBasedTransportDescriptor : public eprosima::fastdds::rtps::[TransportDescriptorInterface](transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE)

Base class for all port based transport descriptors

This class provides a common thread settings configuration for all port based transport descriptor implementations 

Subclassed by eprosima::fastdds::rtps::EthernetTransportDescriptor, [eprosima::fastdds::rtps::SharedMemTransportDescriptor](../shm_transport/shared_mem_transport_descriptor.md#structeprosima_1_1fastdds_1_1rtps_1_1SharedMemTransportDescriptor), [eprosima::fastdds::rtps::SocketTransportDescriptor](socket_transport_descriptor.md#structeprosima_1_1fastdds_1_1rtps_1_1SocketTransportDescriptor)

### Public Functions

### PortBasedTransportDescriptor(std::uint32_t maximumMessageSize, std::uint32_t maximumInitialPeersRange)

Constructor. 

### PortBasedTransportDescriptor(const [PortBasedTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps28PortBasedTransportDescriptor28PortBasedTransportDescriptorERK28PortBasedTransportDescriptor) &t) = default

Copy constructor. 

### [PortBasedTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps28PortBasedTransportDescriptorE) &operator=(const [PortBasedTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps28PortBasedTransportDescriptorE) &t) = default

Copy assignment. 

### virtual ~PortBasedTransportDescriptor() = default

Destructor. 

### bool operator==(const [PortBasedTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps28PortBasedTransportDescriptorE) &t) const

Comparison operator. 

### virtual const [ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &get_thread_config_for_port(std::uint32_t port) const

Get the [ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) for a specific port. 

This function first looks for the port-specific [ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) in the user-configured reception threads map, i.e. the collection of [ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) returned by [reception_threads()](#classeprosima_1_1fastdds_1_1rtps_1_1PortBasedTransportDescriptor_1afff219ebf714a488b2432bf22e3db53a). If the [ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) are found within said map, then [get_thread_config_for_port()](#classeprosima_1_1fastdds_1_1rtps_1_1PortBasedTransportDescriptor_1aab661ccdd9f6957b522716c041f14447) returns them; else it returns the default reception thread settings, i.e. the [ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) returned by [default_reception_threads()](#classeprosima_1_1fastdds_1_1rtps_1_1PortBasedTransportDescriptor_1ac41985eb58dc0f65862ab92d02da0fe0).

#### WARNING
This function will return the default reception thread [ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) when called with a non-default, non-user-configured port.

* **Parameters:**
  **port** – The port to which the returned ThreadSetting apply.
* **Returns:**
  The [ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) for the given port. 

### const [ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &default_reception_threads() const

Returns the [ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) for the default reception threads. 

### virtual void default_reception_threads(const [ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) &default_reception_threads)

Set the [ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) for the default reception threads. 

### const ReceptionThreadsConfigMap &reception_threads() const

Returns the [ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) for the user-configured reception threads. 

### virtual bool reception_threads(const ReceptionThreadsConfigMap &reception_threads)

Set the [ThreadSettings](../../rtps/attributes/ThreadSettings.md#structeprosima_1_1fastdds_1_1rtps_1_1ThreadSettings) for the user-configured reception threads.
