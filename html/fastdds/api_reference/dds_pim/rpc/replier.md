<a id="api-pim-replier"></a>

<a id="replier"></a>

# Replier

### class Replier : public eprosima::fastdds::dds::rpc::[RPCEntity](rpcentity.md#_CPPv4N8eprosima7fastdds3dds3rpc9RPCEntityE)

Base class for a [Replier](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Replier) in the RPC communication. 

### Public Functions

### virtual const std::string &get_service_name() const = 0

Returns the name of the service to which the replier belongs. 

### virtual ReturnCode_t send_reply(void \*data, const [RequestInfo](requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE) &info) = 0

Send a reply message. 

* **Parameters:**
  * **data** – Data to send 
  * **info** – Information about the reply sample. This information is used to match the reply with the request through the SampleIdentity
* **Returns:**
  RETCODE_OK if the reply was sent successfully 
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the replier is not enabled 
* **Returns:**
  RETCODE_NO_DATA if the requester that sent the request has disconnected (this usually means that the reply can be dropped) 
* **Returns:**
  RETCODE_TIMEOUT if waiting for the requester to be fully matched timed out 
* **Returns:**
  a ReturnCode from the underlying [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)

### virtual ReturnCode_t take_request(void \*data, [RequestInfo](requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE) &info) = 0

Take a request message from the [Replier](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Replier) [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)’s history. 

* **Parameters:**
  * **data** – Data to receive the request 
  * **info** – Information about the request sample 
* **Returns:**
  RETCODE_OK if the request was taken successfully or a ReturnCode related to the specific error otherwise 

### virtual ReturnCode_t take_request([LoanableCollection](../core/loanablecollection.md#_CPPv4N8eprosima7fastdds3dds18LoanableCollectionE) &data, [LoanableSequence](../core/loanablesequence.md#_CPPv4I00EN8eprosima7fastdds3dds16LoanableSequenceE)<[RequestInfo](requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE)> &info) = 0

Take all request messages stored in the [Replier](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Replier) [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)’s history. 

#### NOTE
This method does not allow to take only the samples associated to a given request. User must implement a zero-copy solution to link request and reply samples.

* **Parameters:**
  * **data** – Data to receive the request 
  * **info** – Information about the request sample 
* **Returns:**
  RETCODE_OK if the request was taken successfully or a ReturnCode related to the specific error otherwise 

### virtual ReturnCode_t return_loan([LoanableCollection](../core/loanablecollection.md#_CPPv4N8eprosima7fastdds3dds18LoanableCollectionE) &data, [LoanableSequence](../core/loanablesequence.md#_CPPv4I00EN8eprosima7fastdds3dds16LoanableSequenceE)<[RequestInfo](requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE)> &info) = 0

This operation indicates to the [Replier](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Replier)’s [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) that the application is done accessing the collection of Request `datas` and `infos` obtained by some earlier invocation of [take_request](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Replier_1af9a9d4bdddc6856d8e396c092aeb9f30). 

* **Parameters:**
  * **data** – **[inout]** A [LoanableCollection](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection) object where the received data samples were obtained from an earlier invocation of take_request on this [Replier](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Replier). 
  * **info** – **[inout]** A [LoanableSequence](../core/loanablesequence.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableSequence) where the received request infos were obtained from an earlier invocation of take_request on this [Replier](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Replier). 

### virtual [DataWriter](../publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) \*get_replier_writer() const = 0

Getter for the [Replier](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Replier)’s [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). 

### virtual [DataReader](../subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) \*get_replier_reader() const = 0

Getter for the [Replier](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Replier)’s [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader).
