<a id="api-chaining-transport"></a>

<a id="chainingtransport"></a>

# ChainingTransport

### class ChainingTransport : public eprosima::fastdds::rtps::[TransportInterface](../transport_generic_interfaces/transport_interface.md#_CPPv4N8eprosima7fastdds4rtps18TransportInterfaceE)

This is the base class for chaining adapter transports.
* Directly proxies all operations except Send and Receive
* Has a pointer to the low level transport 

### Public Functions

### inline ChainingTransport(const [ChainingTransportDescriptor](chaining_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps27ChainingTransportDescriptorE) &t)

Constructor. 

### ~ChainingTransport() override = default

Destructor. 

### inline virtual bool init(const fastdds::rtps::[PropertyPolicy](../../rtps/attributes/PropertyPolicy.md#_CPPv4N8eprosima7fastdds4rtps14PropertyPolicyE) \*properties = nullptr, const std::uint32_t &max_msg_size_no_frag = 0) override

Initialize the low-level transport. 

This method will prepare all the internals of the transport. 

* **Parameters:**
  * **properties** – Optional policy to specify additional parameters of the created transport. 
  * **max_msg_size_no_frag** – Optional maximum message size to avoid 65500 KB fragmentation limit. 
* **Returns:**
  True when the transport was correctly initialized. 

### inline virtual bool IsInputChannelOpen(const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &loc) const override

Call the low-level transport `[IsInputChannelOpen()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a7797dd9fe7d381d69a74e673b1a44234)`. 

Must report whether the input channel associated to this locator is open. Channels must either be fully closed or fully open, so that “open” and “close” operations are whole and definitive. 

### inline virtual bool IsLocatorSupported(const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &loc) const override

Call the low-level transport `[IsLocatorSupported()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a73b72134bd8eee7db822546aa27aa5df)`. 

Must report whether the given locator is supported by this transport (typically inspecting its “kind” value). 

### inline virtual fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) RemoteToMainLocal(const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &loc) const override

Call the low-level transport `[RemoteToMainLocal()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a646ac507fed76879735e8d8019d691f7)`. 

Returns the locator describing the main (most general) channel that can write to the provided remote locator. 

### virtual bool OpenInputChannel(const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &loc, [TransportReceiverInterface](../transport_generic_interfaces/transport_receiver_interface.md#_CPPv4N8eprosima7fastdds4rtps26TransportReceiverInterfaceE) \*receiver_interface, std::uint32_t max_message_size) override

Call the low-level transport `[OpenInputChannel()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a1d01e84a25020d6fa67f764ec1254764)`. 

Opens an input channel to receive incoming connections. If there is an existing channel it registers the receiver interface. 

### virtual bool OpenOutputChannel([SendResourceList](../transport_generic_interfaces/transport_interface.md#_CPPv4N8eprosima7fastdds4rtps16SendResourceListE) &sender_resource_list, const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &loc) override

Call the low-level transport `[OpenOutputChannel()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a09a1c6addbd9f4e46e244b2c3a5c55b0)`. 

Must open the channel that maps to/from the given locator. This method must allocate, reserve and mark any resources that are needed for said channel. 

### inline virtual bool CloseInputChannel(const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &loc) override

Call the low-level transport `[CloseInputChannel()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a8b6ef3183f9202e045021cc74cd868f0)`. 

Must close the channel that maps to/from the given locator. IMPORTANT: It MUST be safe to call this method even during a Receive operation on another thread. You must implement any necessary mutual exclusion and timeout mechanisms to make sure the channel can be closed without damage. 

### inline virtual fastdds::rtps::LocatorList_t NormalizeLocator(const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &locator) override

Call the low-level transport `[NormalizeLocator()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1ad5231789a435ce6e00e0a2bf6a5824f1)`. 

Performs locator normalization (assign valid IP if not defined by user) 

### inline virtual bool is_local_locator(const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &locator) const override

Call the low-level transport `[is_local_locator()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1adbe010dadbdf7722be2366d878a058e9)`. 

Must report whether the given locator is from the local host 

### inline virtual bool is_localhost_allowed() const override

Call the low-level transport `[is_localhost_allowed()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a129b0ddf6372eae41a3e57f406557a86)`. 

Must report whether localhost locator is allowed 

### inline virtual NetmaskFilterInfo netmask_filter_info() const override

Call the low-level transport `[netmask_filter_info()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1ac83eb520fbc17a93350e1d7f6841ed4c)`. 

Returns netmask filter information (transport’s netmask filter kind and allowlist) 

### inline virtual bool DoInputLocatorsMatch(const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &locator_1, const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &locator_2) const override

Call the low-level transport `[DoInputLocatorsMatch()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a02f19ab045f70d26fea90cc7850fe0c3)`. 

Must report whether two locators map to the same internal channel. 

### inline virtual void select_locators(fastdds::rtps::[LocatorSelector](../../rtps/common/LocatorSelector/LocatorSelector.md#_CPPv4N8eprosima7fastdds4rtps15LocatorSelectorE) &selector) const override

Call the low-level transport `[select_locators()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1afbcf36e93842424067bf98acac544ac8)`. 

Performs the locator selection algorithm for this transport. 

### inline virtual void AddDefaultOutputLocator(fastdds::rtps::LocatorList_t &defaultList) override

Call the low-level transport `[AddDefaultOutputLocator()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1afa7081da8dc2249ff919c1c7502be180)`. 

Add default output locator to the locator list 

### inline virtual bool getDefaultMetatrafficMulticastLocators(fastdds::rtps::LocatorList_t &locators, std::uint32_t metatraffic_multicast_port) const override

Call the low-level transport `[getDefaultMetatrafficMulticastLocators()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a0b5494346ca412df958e2897dd8f4fd4)`. 

Add metatraffic multicast locator with the given port 

### inline virtual bool getDefaultMetatrafficUnicastLocators(fastdds::rtps::LocatorList_t &locators, std::uint32_t metatraffic_unicast_port) const override

Call the low-level transport `[getDefaultMetatrafficUnicastLocators()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1af43a3aedda95c3e38b1d8100c0a433df)`. 

Add metatraffic unicast locator with the given port 

### inline virtual bool getDefaultUnicastLocators(fastdds::rtps::LocatorList_t &locators, std::uint32_t unicast_port) const override

Call the low-level transport `[getDefaultUnicastLocators()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a7aec89799d36aabf8938ea1a693bfdf2)`. 

Add unicast locator with the given port 

### inline virtual bool fillMetatrafficMulticastLocator(fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &locator, std::uint32_t metatraffic_multicast_port) const override

Call the low-level transport `[fillMetatrafficMulticastLocator()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a2b9fa87e13585f68496397749ddf47f0)`. 

Assign port to the given metatraffic multicast locator if not already defined 

### inline virtual bool fillMetatrafficUnicastLocator(fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &locator, std::uint32_t metatraffic_unicast_port) const override

Call the low-level transport `[fillMetatrafficUnicastLocator()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1ae2f0b97f6fbaf3633391d42df275ab81)`. 

Assign port to the given metatraffic unicast locator if not already defined 

### inline virtual bool configureInitialPeerLocator(fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &locator, const fastdds::rtps::[PortParameters](../../rtps/common/PortParameters/PortParameters.md#_CPPv4N8eprosima7fastdds4rtps14PortParametersE) &port_params, std::uint32_t domainId, fastdds::rtps::LocatorList_t &list) const override

Call the low-level transport `[configureInitialPeerLocator()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1aadfd1959c565ca203ea3543ca6858bac)`. 

Configure the initial peer locators list 

### inline virtual bool fillUnicastLocator(fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &locator, std::uint32_t well_known_port) const override

Call the low-level transport `[fillUnicastLocator()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1ac446a343c73654e3fa654057c7ade0ae)`. 

Assign port to the given unicast locator if not already defined 

### inline virtual bool transform_remote_locator(const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &remote_locator, fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &result_locator) const override

Call the low-level transport `[transform_remote_locator()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a029c85ab19621ca7afda8fb532f7f091)`. Transforms a remote locator into a locator optimized for local communications. 

### inline virtual std::uint32_t max_recv_buffer_size() const override

Call the low-level transport `[max_recv_buffer_size()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a18e86582b005909b57ed7b46ad5486ac)`. 

* **Returns:**
  The maximum datagram size for reception supported by the transport 

### virtual bool send(fastdds::rtps::SenderResource \*low_sender_resource, const std::vector<NetworkBuffer> &buffers, std::uint32_t total_bytes, fastdds::rtps::[LocatorsIterator](../../rtps/common/Locator/LocatorsIterator.md#_CPPv4N8eprosima7fastdds4rtps16LocatorsIteratorE) \*destination_locators_begin, fastdds::rtps::[LocatorsIterator](../../rtps/common/Locator/LocatorsIterator.md#_CPPv4N8eprosima7fastdds4rtps16LocatorsIteratorE) \*destination_locators_end, const std::chrono::steady_clock::time_point &timeout) = 0

Blocking Send through the specified channel. It may perform operations on the output buffer. At the end the function must call to the low-level transport’s `[send()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a707cfa66083e3bca36c55b88340d5e59)` function. 
```cpp
// Example of calling the low-level transport `send()` function.
return low_sender_resource->send(buffers, total_bytes, destination_locators_begin,
            destination_locators_end, timeout);
```

 * **Parameters:**
  * **low_sender_resource** – SenderResource generated by the lower transport. 
  * **buffers** – Vector of buffers to send. 
  * **total_bytes** – Length of all buffers to be sent. Will be used as a boundary for the previous parameter. It must not exceed the `sendBufferSize` fed to this class during construction. 
  * **destination_locators_begin** – First iterator of the list of [Locators](../../rtps/common/Locator/Locators.md#classeprosima_1_1fastdds_1_1rtps_1_1Locators) describing the remote destinations we’re sending to. 
  * **destination_locators_end** – End iterator of the list of [Locators](../../rtps/common/Locator/Locators.md#classeprosima_1_1fastdds_1_1rtps_1_1Locators) describing the remote destinations we’re sending to. 
  * **timeout** – Maximum blocking time. 

### inline virtual bool send_w_priority(fastdds::rtps::SenderResource \*low_sender_resource, const std::vector<NetworkBuffer> &buffers, std::uint32_t total_bytes, fastdds::rtps::[LocatorsIterator](../../rtps/common/Locator/LocatorsIterator.md#_CPPv4N8eprosima7fastdds4rtps16LocatorsIteratorE) \*destination_locators_begin, fastdds::rtps::[LocatorsIterator](../../rtps/common/Locator/LocatorsIterator.md#_CPPv4N8eprosima7fastdds4rtps16LocatorsIteratorE) \*destination_locators_end, const std::chrono::steady_clock::time_point &timeout, std::int32_t transport_priority)

Blocking Send through the specified channel. It may perform operations on the output buffer. At the end the function must call to the low-level transport’s `[send()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a707cfa66083e3bca36c55b88340d5e59)` function. 
```cpp
// Example of calling the low-level transport `send()` function.
return low_sender_resource->send(buffers, total_bytes, destination_locators_begin,
            destination_locators_end, timeout, transport_priority);
```

 * **Parameters:**
  * **low_sender_resource** – SenderResource generated by the lower transport. 
  * **buffers** – Vector of buffers to send. 
  * **total_bytes** – Length of all buffers to be sent. Will be used as a boundary for the previous parameter. It must not exceed the `sendBufferSize` fed to this class during construction. 
  * **destination_locators_begin** – First iterator of the list of [Locators](../../rtps/common/Locator/Locators.md#classeprosima_1_1fastdds_1_1rtps_1_1Locators) describing the remote destinations we’re sending to. 
  * **destination_locators_end** – End iterator of the list of [Locators](../../rtps/common/Locator/Locators.md#classeprosima_1_1fastdds_1_1rtps_1_1Locators) describing the remote destinations we’re sending to. 
  * **timeout** – Maximum blocking time. 
  * **transport_priority** – Transport priority to be used for the send operation. 

### virtual void receive([TransportReceiverInterface](../transport_generic_interfaces/transport_receiver_interface.md#_CPPv4N8eprosima7fastdds4rtps26TransportReceiverInterfaceE) \*next_receiver, const fastdds::rtps::[octet](../../rtps/common/Types/octet.md#_CPPv4N8eprosima7fastdds4rtps5octetE) \*receive_buffer, std::uint32_t receive_buffer_size, const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &local_locator, const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &remote_locator) = 0

Blocking Receive from the specified channel. 

It may perform operations on the input buffer. At the end the function must call to the `next_receiver`’s `OnDataReceived` function. 
```cpp
// Example of calling the `next_receiver`'s `OnDataReceived` function.
next_receiver->OnDataReceived(receive_buffer, receive_buffer_size, local_locator, remote_locator);
```

 * **Parameters:**
  * **next_receiver** – Next resource receiver to be called. 
  * **receive_buffer** – vector with enough capacity (not size) to accommodate a full receive buffer. That capacity must not be less than the `receiveBufferSize` supplied to this class during construction. 
  * **receive_buffer_size** – Size of the raw data. It will be used as bounds check for the previous argument. It must not exceed the `receiveBufferSize` fed to this class during construction. 
  * **local_locator** – Locator mapping to the local channel we’re listening to. 
  * **remote_locator** – **[out]** Locator describing the remote destination we received a packet from. 

### inline virtual void update_network_interfaces() override

Update network interfaces. 

### inline virtual bool transform_remote_locator(const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &remote_locator, fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &result_locator, bool allowed_remote_localhost, bool allowed_local_localhost) const override

Call the low-level transport `[transform_remote_locator()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a029c85ab19621ca7afda8fb532f7f091)`. Transforms a remote locator into a locator optimized for local communications, if allowed by both local and remote transports. 

### inline virtual bool is_locator_allowed(const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &locator) const override

Call the low-level transport `[is_locator_allowed()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1ab73cc5b02b743450b2accb16053560f4)`. 

Must report whether the given locator is allowed by this transport. 

### inline virtual bool is_locator_reachable(const fastdds::rtps::[Locator_t](../../rtps/common/Locator/Locator_t.md#_CPPv4N8eprosima7fastdds4rtps9Locator_tE) &locator) override

Call the low-level transport `[is_locator_reachable()](#classeprosima_1_1fastdds_1_1rtps_1_1ChainingTransport_1a2e66d42658842eddac10af66b59484ba)`. 

Must report whether the given locator is reachable by this transport.
