<a id="api-transport-interface"></a>

<a id="transportinterface"></a>

# TransportInterface

### class TransportInterface

Interface against which to implement a transport layer, decoupled from Fast DDS internals. [TransportInterface](#classeprosima_1_1fastdds_1_1rtps_1_1TransportInterface) expects the user to implement a logical equivalence between [Locators](../../rtps/common/Locator/Locators.md#classeprosima_1_1fastdds_1_1rtps_1_1Locators) and protocol-specific “channels”. This equivalence can be narrowing: For instance in UDP/IP, a port would take the role of channel, and several different locators can map to the same port, and hence the same channel. 

Subclassed by [eprosima::fastdds::rtps::ChainingTransport](../chaining_transport/chaining_transport.md#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport)

### Public Functions

### virtual ~TransportInterface() = default

Aside from the API defined here, an user-defined Transport must define a descriptor data type and a constructor that expects a constant reference to such descriptor. e.g:

class MyTransport: public: typedef struct { … } MyTransportDescriptor; MyTransport(const MyTransportDescriptor&); … 

### TransportInterface(const [TransportInterface](#_CPPv4N8eprosima7fastdds4rtps18TransportInterface18TransportInterfaceERK18TransportInterface) &t) = delete

Copy constructor. 

### [TransportInterface](#_CPPv4N8eprosima7fastdds4rtps18TransportInterfaceE) &operator=(const [TransportInterface](#_CPPv4N8eprosima7fastdds4rtps18TransportInterfaceE) &t) = delete

Copy assignment. 

### TransportInterface([TransportInterface](#_CPPv4N8eprosima7fastdds4rtps18TransportInterface18TransportInterfaceERR18TransportInterface) &&t) = delete

Move constructor. 

### [TransportInterface](#_CPPv4N8eprosima7fastdds4rtps18TransportInterfaceE) &operator=([TransportInterface](#_CPPv4N8eprosima7fastdds4rtps18TransportInterfaceE) &&t) = delete

Move assignment. 

### virtual bool init(const fastdds::rtps::[PropertyPolicy](../../rtps/attributes/PropertyPolicy.md#_CPPv4N8eprosima7fastdds4rtps14PropertyPolicyE) \*properties = nullptr, const std::uint32_t &max_msg_size_no_frag = 0) = 0

Initialize this transport. This method will prepare all the internals of the transport. 

* **Parameters:**
  * **properties** – Optional policy to specify additional parameters of the created transport. 
  * **max_msg_size_no_frag** – Optional maximum message size to avoid 65500 KB fragmentation limit. 
* **Returns:**
  True when the transport was correctly initialized. 

### virtual bool IsInputChannelOpen(const Locator&) const = 0

Must report whether the input channel associated to this locator is open. Channels must either be fully closed or fully open, so that “open” and “close” operations are whole and definitive. 

### virtual bool IsLocatorSupported(const Locator&) const = 0

Must report whether the given locator is supported by this transport (typically inspecting its “kind” value). 

### virtual bool is_locator_allowed(const Locator&) const = 0

Must report whether the given locator is allowed by this transport. 

### virtual bool is_locator_reachable(const [Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &locator) = 0

Must report whether the given locator is reachable by this transport.

* **Parameters:**
  **locator** – **[in]** Locator for which the reachability is checked.
* **Returns:**
  true if the input locator is reachable by this transport, false otherwise. 

### virtual Locator RemoteToMainLocal(const Locator &remote) const = 0

Returns the locator describing the main (most general) channel that can write to the provided remote locator. 

### inline virtual bool transform_remote_locator(const Locator &remote_locator, Locator &result_locator) const

Transforms a remote locator into a locator optimized for local communications.

If the remote locator corresponds to one of the local interfaces, it is converted to the corresponding local address.

* **Parameters:**
  * **remote_locator** – **[in]** Locator to be converted. 
  * **result_locator** – **[out]** Converted locator.
* **Returns:**
  false if the input locator is not supported/allowed by this transport, true otherwise. 

### virtual bool OpenOutputChannel([SendResourceList](#_CPPv4N8eprosima7fastdds4rtps16SendResourceListE) &sender_resource_list, const Locator&) = 0

Must open the channel that maps to/from the given locator. This method must allocate, reserve and mark any resources that are needed for said channel. 

### virtual bool OpenOutputChannels([SendResourceList](#_CPPv4N8eprosima7fastdds4rtps16SendResourceListE) &sender_resource_list, const fastdds::rtps::[LocatorSelectorEntry](../../rtps/common/LocatorSelectorEntry/LocatorSelectorEntry.md#_CPPv4N8eprosima7fastdds4rtps20LocatorSelectorEntryE) &locator_selector_entry)

Must open the channel that maps to/from the given locator selector entry. This method must allocate, reserve and mark any resources that are needed for said channel.

* **Parameters:**
  * **sender_resource_list** – Participant’s send resource list. 
  * **locator_selector_entry** – Locator selector entry with the remote entity locators.
* **Returns:**
  true if the channel was correctly opened or if finding an already opened one. 

### virtual void CloseOutputChannels([SendResourceList](#_CPPv4N8eprosima7fastdds4rtps16SendResourceListE) &sender_resource_list, const fastdds::rtps::[LocatorSelectorEntry](../../rtps/common/LocatorSelectorEntry/LocatorSelectorEntry.md#_CPPv4N8eprosima7fastdds4rtps20LocatorSelectorEntryE) &locator_selector_entry)

Close the channel that maps to/from the given locator selector entry.

* **Parameters:**
  * **sender_resource_list** – Participant’s send resource list. 
  * **locator_selector_entry** – Locator selector entry with the remote entity locators. 

### virtual bool OpenInputChannel(const Locator&, [TransportReceiverInterface](transport_receiver_interface.md#_CPPv4N8eprosima7fastdds4rtps26TransportReceiverInterfaceE)\*, std::uint32_t) = 0

Opens an input channel to receive incoming connections. If there is an existing channel it registers the receiver interface. 

### virtual bool CloseInputChannel(const Locator&) = 0

Must close the channel that maps to/from the given locator. IMPORTANT: It MUST be safe to call this method even during a Receive operation on another thread. You must implement any necessary mutual exclusion and timeout mechanisms to make sure the channel can be closed without damage. 

### virtual bool DoInputLocatorsMatch(const Locator&, const Locator&) const = 0

Must report whether two locators map to the same internal channel. 

### virtual [LocatorList](../../rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) NormalizeLocator(const Locator &locator) = 0

Performs locator normalization (assign valid IP if not defined by user) 

### virtual void select_locators(fastdds::rtps::[LocatorSelector](../../rtps/common/LocatorSelector/LocatorSelector.md#_CPPv4N8eprosima7fastdds4rtps15LocatorSelectorE) &selector) const = 0

Performs the locator selection algorithm for this transport.

It basically consists of the following steps
* selector.transport_starts is called
* transport handles the selection state of each locator
* if a locator from an entry is selected, selector.select is called for that entry

* **Parameters:**
  **selector** – **[inout]** Locator selector. 

### virtual bool is_local_locator(const Locator &locator) const = 0

Must report whether the given locator is from the local host. 

### virtual [TransportDescriptorInterface](transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE) \*get_configuration() = 0

Return the transport configuration (Transport Descriptor) 

### virtual void AddDefaultOutputLocator([LocatorList](../../rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &defaultList) = 0

Add default output locator to the locator list. 

### virtual bool getDefaultMetatrafficMulticastLocators([LocatorList](../../rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &locators, std::uint32_t metatraffic_multicast_port) const = 0

Add metatraffic multicast locator with the given port. 

### virtual bool getDefaultMetatrafficUnicastLocators([LocatorList](../../rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &locators, std::uint32_t metatraffic_unicast_port) const = 0

Add metatraffic unicast locator with the given port. 

### virtual bool getDefaultUnicastLocators([LocatorList](../../rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &locators, std::uint32_t unicast_port) const = 0

Add unicast locator with the given port. 

### virtual bool fillMetatrafficMulticastLocator(Locator &locator, std::uint32_t metatraffic_multicast_port) const = 0

Assign port to the given metatraffic multicast locator if not already defined. 

### virtual bool fillMetatrafficUnicastLocator(Locator &locator, std::uint32_t metatraffic_unicast_port) const = 0

Assign port to the given metatraffic unicast locator if not already defined. 

### virtual bool configureInitialPeerLocator(Locator &locator, const fastdds::rtps::[PortParameters](../../rtps/common/PortParameters/PortParameters.md#_CPPv4N8eprosima7fastdds4rtps14PortParametersE) &port_params, std::uint32_t domainId, [LocatorList](../../rtps/common/Locator/LocatorList.md#_CPPv4N8eprosima7fastdds4rtps11LocatorListE) &list) const = 0

Configure the initial peer locators list. 

### virtual bool fillUnicastLocator(Locator &locator, std::uint32_t well_known_port) const = 0

Assign only the port to the given unicast locator if not already defined Note: As the default address is usually valid for unicast, this method does not assign a new value 

### virtual std::uint32_t max_recv_buffer_size() const = 0

* **Returns:**
  The maximum datagram size for reception supported by the transport 

### inline virtual void shutdown()

Shutdown method to close the connections of the transports. 

### inline virtual void update_network_interfaces()

Update network interfaces. 

### inline std::int32_t kind() const

Return the transport kind. 

### inline virtual bool transform_remote_locator(const Locator &remote_locator, Locator &result_locator, bool allowed_remote_localhost, bool allowed_local_localhost) const

Transforms a remote locator into a locator optimized for local communications.

If the remote locator corresponds to one of the local interfaces, it is converted to the corresponding local address if allowed by both local and remote transports.

* **Parameters:**
  * **remote_locator** – **[in]** Locator to be converted. 
  * **result_locator** – **[out]** Converted locator. 
  * **allowed_remote_localhost** – **[in]** Whether localhost is allowed (and hence used) in the remote transport. 
  * **allowed_local_localhost** – **[in]** Whether localhost is allowed locally (by this or other transport).
* **Returns:**
  false if the input locator is not supported/allowed by this transport, true otherwise. 

### inline virtual bool is_localhost_allowed() const

Must report whether localhost locator is allowed. 

### inline virtual NetmaskFilterInfo netmask_filter_info() const

Returns netmask filter information (transport’s netmask filter kind and allowlist) 

### constexpr std::uint32_t eprosima::fastdds::rtps::s_maximumMessageSize = 65500

Default maximum message size. 

### constexpr std::uint32_t eprosima::fastdds::rtps::s_maximumInitialPeersRange = 4

Default maximum initial peers range. 

### static const std::string eprosima::fastdds::rtps::s_IPv4AddressAny = "0.0.0.0"

Default IPv4 address. 

### static const std::string eprosima::fastdds::rtps::s_IPv6AddressAny = "::"

Default IPv6 address. 

### using eprosima::fastdds::rtps::SendResourceList = std::vector<std::unique_ptr<fastdds::rtps::SenderResource>>
