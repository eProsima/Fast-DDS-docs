<a id="python-api-pim-durabilityqospolicykind"></a>

<a id="durabilityqospolicykind"></a>

# DurabilityQosPolicyKind

### fastdds.VOLATILE_DURABILITY_QOS

The Service does not need to keep any samples of data-instances on behalf of any DataReader that is not
known by the DataWriter at the time the instance is written. In other words the Service will only attempt
to provide the data to existing subscribers

### fastdds.TRANSIENT_LOCAL_DURABILITY_QOS

For TRANSIENT_LOCAL, the service is only required to keep the data in the memory of the DataWriter that
wrote the data and the data is not required to survive the DataWriter.

### fastdds.TRANSIENT_DURABILITY_QOS

For TRANSIENT, the service is only required to keep the data in memory and not in permanent storage; but
the data is not tied to the lifecycle of the DataWriter and will, in general, survive it.

### fastdds.PERSISTENT_DURABILITY_QOS

Data is kept on permanent storage, so that they can outlive a system session.

Warning: Not Supported
