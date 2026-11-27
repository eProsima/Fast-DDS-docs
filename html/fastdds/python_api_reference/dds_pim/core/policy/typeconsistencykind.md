<a id="python-api-pim-typeconsistencykind"></a>

<a id="typeconsistencykind"></a>

# TypeConsistencyKind

### fastdds.DISALLOW_TYPE_COERCION

The DataWriter and the DataReader must support the same data type in order for them to communicate.

### fastdds.ALLOW_TYPE_COERCION

The DataWriter and the DataReader need not support the same data type in order for them to communicate as long as
the reader’s type is assignable from the writer’s type.
