<a id="python-api-pim-timebasedfilterqospolicy"></a>

<a id="timebasedfilterqospolicy"></a>

# TimeBasedFilterQosPolicy

### *class* fastdds.TimeBasedFilterQosPolicy

Filter that allows a DataReader to specify that it is interested only in (potentially) a subset of the values of the data.
The filter states that the DataReader does not want to receive more than one value each minimum_separation, regardless
of how fast the changes occur. It is inconsistent for a DataReader to have a minimum_separation longer than its
Deadline period.

Warning: This QosPolicy can be defined and is transmitted to the rest of the network but is not implemented in this version.

Notes: Mutable Qos Policy

#### clear()

Clears the QosPolicy object

#### *property* minimum_separation

Minimum interval between samples. By default, dds::c_TimeZero (the DataReader is interested in all values)

#### *property* thisown

The membership flag
