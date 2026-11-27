<a id="api-pim-requester"></a>

<a id="requester"></a>

# Requester

### class Requester : public eprosima::fastdds::dds::rpc::[RPCEntity](rpcentity.md#_CPPv4N8eprosima7fastdds3dds3rpc9RPCEntityE)

Base class for a [Requester](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Requester) in the RPC communication. 

### Public Functions

### virtual const std::string &get_service_name() const = 0

Returns the name of the service to which the requester belongs. 

### virtual ReturnCode_t send_request(void \*data, [RequestInfo](requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE) &info) = 0

Send a request message. 

* **Parameters:**
  * **data** – Data to send 
  * **info** – Information about the request sample. This information is used to match the request with the reply through the SampleIdentity
* **Returns:**
  RETCODE_OK if the reply was sent successfully 
* **Returns:**
  RETCODE_PRECONDITION_NOT_MET if the requester is not enabled or it is not fully matched 
* **Returns:**
  a ReturnCode from the underlying [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter)

### virtual ReturnCode_t take_reply(void \*data, [RequestInfo](requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE) &info) = 0

Take a reply message from the [Requester](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Requester) [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)’s history. 

* **Parameters:**
  * **data** – Data to receive the reply 
  * **info** – Information about the reply sample 
* **Returns:**
  RETCODE_OK if the reply was taken successfully or a ReturnCode related to the specific error otherwise 

### virtual ReturnCode_t take_reply([LoanableCollection](../core/loanablecollection.md#_CPPv4N8eprosima7fastdds3dds18LoanableCollectionE) &data, [LoanableSequence](../core/loanablesequence.md#_CPPv4I00EN8eprosima7fastdds3dds16LoanableSequenceE)<[RequestInfo](requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE)> &info) = 0

Take all reply messages stored in the [Requester](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Requester) [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)’s history. 

#### NOTE
This method does not allow to take only the samples associated to a given request. User must implement a zero-copy solution to link request and reply samples.

* **Parameters:**
  * **data** – Data to receive the replies 
  * **info** – Information about the reply samples 
* **Returns:**
  RETCODE_OK if the replies were taken successfully or a ReturnCode related to the specific error otherwise 

### virtual ReturnCode_t return_loan([LoanableCollection](../core/loanablecollection.md#_CPPv4N8eprosima7fastdds3dds18LoanableCollectionE) &data, [LoanableSequence](../core/loanablesequence.md#_CPPv4I00EN8eprosima7fastdds3dds16LoanableSequenceE)<[RequestInfo](requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE)> &info) = 0

This operation indicates to the [Requester](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Requester)’s [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) that the application is done accessing the collection of Reply `datas` and `infos` obtained by some earlier invocation of [take_reply](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Requester_1a7f5732bee50c6bab2cb79be9477789ec). 

* **Parameters:**
  * **data** – **[inout]** A [LoanableCollection](../core/loanablecollection.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableCollection) object where the received data samples were obtained from an earlier invocation of take_reply on this [Requester](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Requester). 
  * **info** – **[inout]** A [LoanableSequence](../core/loanablesequence.md#classeprosima_1_1fastdds_1_1dds_1_1LoanableSequence) where the received request infos were obtained from an earlier invocation of take_reply on this [Requester](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Requester). 

### virtual [DataWriter](../publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) \*get_requester_writer() const = 0

Getter for the [Requester](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Requester)’s [DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter). 

### virtual [DataReader](../subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) \*get_requester_reader() const = 0

Getter for the [Requester](#classeprosima_1_1fastdds_1_1dds_1_1rpc_1_1Requester)’s [DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader).
