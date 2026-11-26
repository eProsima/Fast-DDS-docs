<a id="python-api-pim-datarepresentationqospolicy"></a>

<a id="datarepresentationqospolicy"></a>

# DataRepresentationQosPolicy

### *class* fastdds.DataRepresentationQosPolicy

With multiple standard data Representations available, and vendor-specific extensions possible, DataWriters and
DataReaders must be able to negotiate which data representation(s) to use. This negotiation shall occur based on
DataRepresentationQosPolicy.

Warning: If a writer’s offered representation is contained within a reader’s sequence, the offer satisfies the
request and the policies are compatible. Otherwise, they are incompatible.

Notes: Immutable Qos Policy

#### clear()

Clears the QosPolicy object

#### *property* m_value

List of ‘DataRepresentationId’. 
: By default, empty list.

#### *property* thisown

The membership flag
