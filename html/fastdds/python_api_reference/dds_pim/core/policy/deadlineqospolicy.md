<a id="python-api-pim-deadlineqospolicy"></a>

<a id="deadlineqospolicy"></a>

# DeadlineQosPolicy

### *class* fastdds.DeadlineQosPolicy

DataReader expects a new sample updating the value of each instance at least once every deadline period.
DataWriter indicates that the application commits to write a new value (using the DataWriter) for each instance managed
by the DataWriter at least once every deadline period.

Notes: Mutable Qos Policy

#### clear()

Clears the QosPolicy object

#### *property* period

Maximum time expected between samples.
It is inconsistent for a DataReader to have a DEADLINE period less than its TimeBasedFilterQosPolicy
minimum_separation.

By default, c_TimeInifinite.

#### *property* thisown

The membership flag
