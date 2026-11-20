<a id="api-pim-replierqos"></a>

<a id="replierqos"></a>

# ReplierQos

### class ReplierQos

### Public Functions

### inline ReplierQos()

Constructor. 

### inline bool operator==(const [ReplierQos](#_CPPv4N8eprosima7fastdds3dds10ReplierQosE) &b) const

Equal comparison operator. 

### Public Members

### std::string service_name

Service name. 

### std::string request_type

Request type. 

### std::string reply_type

Reply type. 

### std::string request_topic_name

Request topic name. 

### std::string reply_topic_name

Reply topic name. 

### [DataWriterQos](../publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) writer_qos

[DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) QoS for the reply writer. 

### [DataReaderQos](../subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) reader_qos

[DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) QoS for the reply reader.
