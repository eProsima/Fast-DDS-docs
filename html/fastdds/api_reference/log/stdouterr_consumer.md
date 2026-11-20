<a id="api-log-stdouterr-consumer"></a>

<a id="stdouterrconsumer"></a>

# StdoutErrConsumer

### class StdoutErrConsumer : public eprosima::fastdds::dds::[OStreamConsumer](ostream_consumer.md#_CPPv4N8eprosima7fastdds3dds15OStreamConsumerE)

### Public Functions

### virtual void stderr_threshold(const [Log](log_class.md#_CPPv4N8eprosima7fastdds3dds3LogE)::[Kind](log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4KindE) &kind)

Set the stderr_threshold to a [Log::Kind](log_class.md#classeprosima_1_1fastdds_1_1dds_1_1Log_1a62a215617432d12879894f2b0d32a81d). This threshold decides which log messages are output on STDOUT, and which are output to STDERR. [Log](log_class.md#classeprosima_1_1fastdds_1_1dds_1_1Log) messages with a [Log::Kind](log_class.md#classeprosima_1_1fastdds_1_1dds_1_1Log_1a62a215617432d12879894f2b0d32a81d) equal to or more severe than the stderr_threshold are output to STDERR using std::cerr. [Log](log_class.md#classeprosima_1_1fastdds_1_1dds_1_1Log) messages with a [Log::Kind](log_class.md#classeprosima_1_1fastdds_1_1dds_1_1Log_1a62a215617432d12879894f2b0d32a81d) less severe than the stderr_threshold are output to STDOUT using std::cout. 

* **Parameters:**
  **kind** – The [Log::Kind](log_class.md#classeprosima_1_1fastdds_1_1dds_1_1Log_1a62a215617432d12879894f2b0d32a81d) to which stderr_threshold is set. 

### virtual [Log](log_class.md#_CPPv4N8eprosima7fastdds3dds3LogE)::[Kind](log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4KindE) stderr_threshold() const

Retrieve the stderr_threshold. 

* **Returns:**
  The [Log::Kind](log_class.md#classeprosima_1_1fastdds_1_1dds_1_1Log_1a62a215617432d12879894f2b0d32a81d) to which stderr_threshold is set. 

### Public Static Attributes

### static constexpr [Log](log_class.md#_CPPv4N8eprosima7fastdds3dds3LogE)::[Kind](log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4KindE) STDERR_THRESHOLD_DEFAULT = [Log](log_class.md#_CPPv4N8eprosima7fastdds3dds3LogE)::[Kind](log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4KindE)::[Warning](log_class.md#_CPPv4N8eprosima7fastdds3dds3Log4Kind7WarningE)

Default value of stderr_threshold.
