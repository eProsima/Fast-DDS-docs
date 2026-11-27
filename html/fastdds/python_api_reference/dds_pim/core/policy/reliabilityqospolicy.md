<a id="python-api-pim-reliabilityqospolicy"></a>

<a id="reliabilityqospolicy"></a>

# ReliabilityQosPolicy

### *class* fastdds.ReliabilityQosPolicy

Indicates the reliability of the endpoint.

Notes: Immutable Qos Policy

#### clear()

Clears the QosPolicy object

#### *property* kind

Defines the reliability kind of the endpoint.

By default, BEST_EFFORT_RELIABILITY_QOS for DataReaders and RELIABLE_RELIABILITY_QOS for DataWriters.

#### *property* max_blocking_time

Defines the maximum period of time certain methods will be blocked.

Methods affected by this property are:
- DataWriter::write
- DataReader::takeNextData
- DataReader::readNextData

By default, 100 ms.

#### *property* thisown

The membership flag
