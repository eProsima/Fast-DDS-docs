<a id="discoveryprotocol"></a>

# DiscoveryProtocol

### enum class eprosima::fastdds::rtps::DiscoveryProtocol

PDP subclass choice. 

*Values:*

### enumerator NONE

NO discovery whatsoever would be used. 

Publisher and Subscriber defined with the same topic name would NOT be linked. All matching must be done manually through the addReaderLocator, addReaderProxy, addWriterProxy methods. 

### enumerator SIMPLE

Discovery works according to ‘The Real-time Publish-Subscribe Protocol(RTPS) DDS Interoperability Wire Protocol Specification’. 

### enumerator EXTERNAL

A user defined PDP subclass object must be provided in the attributes that deals with the discovery. 

Framework is not responsible of this object lifetime. 

### enumerator CLIENT

The participant will behave as a client concerning discovery operation. 

Server locators should be specified as attributes. 

### enumerator SERVER

The participant will behave as a server concerning discovery operation. 

Discovery operation is volatile (discovery handshake must take place if shutdown). 

### enumerator BACKUP

The participant will behave as a server concerning discovery operation. 

Discovery operation persist on a file (discovery handshake wouldn’t repeat if shutdown). 

### enumerator SUPER_CLIENT

The participant will behave as a client concerning all internal behaviour. 

Remote servers will treat it as a server and will share every discovery information. 

### enumerator PDP_RELAY

 **[Pro]**  The participant will only re-distribute DATA(p) among the participants that announce to it, allowing SIMPLE participants to discover each other without multicast. No EDP is run, and no other discovery information is relayed.
