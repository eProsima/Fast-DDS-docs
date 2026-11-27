<a id="api-pim-typeconsistencykind"></a>

<a id="typeconsistencykind"></a>

# TypeConsistencyKind

### enum eprosima::fastdds::dds::TypeConsistencyKind

*Values:*

### enumerator DISALLOW_TYPE_COERCION

The [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) and the [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) must support the same data type in order for them to communicate. 

### enumerator ALLOW_TYPE_COERCION

The [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) and the [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) need not support the same data type in order for them to communicate as long as the reader’s type is assignable from the writer’s type.
