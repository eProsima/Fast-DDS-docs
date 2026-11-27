<a id="api-log-file-consumer"></a>

<a id="fileconsumer"></a>

# FileConsumer

### class FileConsumer : public eprosima::fastdds::dds::[OStreamConsumer](ostream_consumer.md#_CPPv4N8eprosima7fastdds3dds15OStreamConsumerE)

### Public Functions

### FileConsumer()

Default constructor: filename = “output.log”, append = false. 

### FileConsumer(const std::string &filename, bool append = false)

Constructor with parameters. 

* **Parameters:**
  * **filename** – path of the output file where the log will be wrote. 
  * **append** – indicates if the consumer must append the content in the filename.
