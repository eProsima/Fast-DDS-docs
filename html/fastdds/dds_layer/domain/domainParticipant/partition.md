<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="partitions"></a>

<a id="id1"></a>

# Partitions

Partitions introduce a logical entity isolation level concept inside the physical isolation induced by a
[Domain](../domain.md#dds-layer-domain).
They represent another level to separate [Publishers](../../publisher/publisher.md#dds-layer-publisher) and
[Subscribers](../../subscriber/subscriber.md#dds-layer-subscriber) beyond Domain and
[Topic](../../topic/topic.md#dds-layer-topic).
For a Publisher to communicate with a Subscriber,
they have to belong at least to one common partition.
In this sense, partitions represent a light mechanism to provide data separation among endpoints:

* Unlike Domain and Topic, Partitions can be changed dynamically
  during the life cycle of the endpoint with little cost.
  Specifically, no new threads are launched, no new memory is allocated, and the change history is not affected.
  Beware that modifying the Partition membership of endpoints will trigger the announcement
  of the new QoS configuration, and as a result, new endpoint matching may occur,
  depending on the new Partition configuration.
  Changes on the memory allocation and running threads may occur due to the matching of remote endpoints.
* Unlike Domain and Topic, an endpoint can belong to several Partitions
  at the same time.
  For certain data to be shared over different Topics, there must be a different
  Publisher for each Topic,
  each of them sharing its own history of changes.
  On the other hand, a single Publisher can share the same data over different Partitions
  using a single topic data change, thus reducing network overload.

The Partition membership of an endpoint can be configured on the [PartitionQosPolicy](../../../api_reference/dds_pim/core/policy/partitionqospolicy.md#api-pim-partitionqospolicy)
data member of the [PublisherQos](../../publisher/publisher/publisher.md#dds-layer-publisher-publisherqos) or [SubscriberQos](../../subscriber/subscriber/subscriber.md#dds-layer-subscriber-subscriberqos) objects.
This member holds a list of Partition name strings.
In DDS, the default Partition name is the empty string `""`.
Therefore, an empty list (`{}`) is equivalent to a list containing only the empty string (`{""}`).
As a result, a Publisher and a Subscriber that specify no Partition can still communicate through the default
Partition, and explicitly configuring `""` has the same behavior.
If `""` is included together with additional names, the endpoint belongs to the default Partition and to those
additional Partitions.

#### WARNING
Partitions are linked to the endpoint and not to the changes.
This means that the endpoint history is oblivious to modifications in the Partitions.
For example, if a Publisher switches Partitions and afterwards needs to resend some older change again,
it will deliver it to the new Partition set, regardless of which Partitions were defined
when the change was created.
This means that a late joiner Subscriber may receive changes that were created when another
set of Partitions was active.

<a id="wildcards-in-partitions"></a>

## Wildcards in Partitions

Partition name entries can have wildcards following the naming conventions defined by the
POSIX `fnmatch` API ([1003.2-1992 section B.6](https://standards.ieee.org/standard/1003_2-1992.html)).
Entries with wildcards can match several names, allowing an endpoint to easily be included in several Partitions.
Two Partition names with wildcards will match if either of them matches the other one according to `fnmatch`.
That is, the matching is checked both ways.
For example, consider the following configuration:

- A Publisher with Partition `part*`
- A Subscriber with Partition `partition*`

Even though `partition*` does not match `part*`, these Publisher and Subscriber
will communicate between them because `part*` matches `partition*`.

Note that a Partition with name `*` will match any other partition,
including the default Partition `""`.

<a id="full-example"></a>

## Full example

Given a system with the following Partition configuration:

| Participant_1   | Pub_11                | {“Partition_1”, “Partition_2”}   |
|-----------------|-----------------------|----------------------------------|
| Pub_12          | {“\*”}                |                                  |
| Participant_2   | Pub_21                | {default} ≡ {“”} ≡ {}            |
| Pub_22          | {“Partition\*”}       |                                  |
| Participant_3   | Subs_31               | {“Partition_1”}                  |
| Subs_32         | {“Partition_2”}       |                                  |
| Subs_33         | {“Partition_3”}       |                                  |
| Subs_34         | {default} ≡ {“”} ≡ {} |                                  |

The endpoints will finally match the Partitions depicted on the following table.

|             |        |        |        |         | Participant_1   | Participant_2   | Participant_3   |
|-------------|--------|--------|--------|---------|-----------------|-----------------|-----------------|
| Pub_11      | Pub_12 | Pub_21 | Pub_22 | Subs_31 | Subs_32         | Subs_33         | Subs_34         |
| Partition_1 | ✓      | ✓      | ✕      | ✓       | ✓               | ✕               | ✕               |
| Partition_2 | ✓      | ✓      | ✕      | ✓       | ✕               | ✓               | ✕               |
| Partition_3 | ✕      | ✓      | ✕      | ✓       | ✕               | ✕               | ✓               |
| {default}   | ✕      | ✓      | ✓      | ✕       | ✕               | ✕               | ✕               |

The following table provides the communication matrix for the given example:

|               |         | Participant_1   | Participant_2   |
|---------------|---------|-----------------|-----------------|
| Pub_11        | Pub_12  | Pub_21          | Pub_22          |
| Participant_3 | Subs_31 | ✓               | ✓               |
| Subs_32       | ✓       | ✓               | ✕               |
| Subs_33       | ✕       | ✓               | ✕               |
| Subs_34       | ✕       | ✓               | ✓               |

The following piece of code shows the set of parameters needed for the use case depicted in this example.

### C++

```c++
PublisherQos pub_11_qos;
pub_11_qos.partition().push_back("Partition_1");
pub_11_qos.partition().push_back("Partition_2");

PublisherQos pub_12_qos;
pub_12_qos.partition().push_back("*");

PublisherQos pub_21_qos;
//No partitions defined for pub_21

PublisherQos pub_22_qos;
pub_22_qos.partition().push_back("Partition*");

SubscriberQos subs_31_qos;
subs_31_qos.partition().push_back("Partition_1");

SubscriberQos subs_32_qos;
subs_32_qos.partition().push_back("Partition_2");

SubscriberQos subs_33_qos;
subs_33_qos.partition().push_back("Partition_3");

SubscriberQos subs_34_qos;
//No partitions defined for subs_34
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <data_writer profile_name="pub_11">
        <qos>
            <partition>
                <names>
                    <name>Partition_1</name>
                    <name>Partition_2</name>
                </names>
            </partition>
        </qos>
    </data_writer>

    <data_writer profile_name="pub_12">
        <qos>
            <partition>
                <names>
                    <name>*</name>
                </names>
            </partition>
        </qos>
    </data_writer>

    <data_writer profile_name="pub_22">
        <qos>
            <partition>
                <names>
                    <name>Partition*</name>
                </names>
            </partition>
        </qos>
    </data_writer>

    <data_reader profile_name="subs_31">
        <qos>
            <partition>
                <names>
                    <name>Partition_1</name>
                </names>
            </partition>
        </qos>
    </data_reader>

    <data_reader profile_name="subs_32">
        <qos>
            <partition>
                <names>
                    <name>Partition_2</name>
                </names>
            </partition>
        </qos>
    </data_reader>

    <data_reader profile_name="subs_33">
        <qos>
            <partition>
                <names>
                    <name>Partition_3</name>
                </names>
            </partition>
        </qos>
    </data_reader>
</profiles>
```
