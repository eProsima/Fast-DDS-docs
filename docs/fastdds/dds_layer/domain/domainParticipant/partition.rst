.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _partitions:

Partitions
==========

Partitions introduce a logical entity isolation level concept inside the physical isolation induced by a
:ref:`dds_layer_domain`.
They represent another level to separate :ref:`Publishers<dds_layer_publisher>` and
:ref:`Subscribers<dds_layer_subscriber>` beyond Domain and
:ref:`dds_layer_topic`.
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

.. |partition| replace:: :cpp:func:`partition<eprosima::fastdds::dds::SubscriberQos::partition>`

The Partition membership of an endpoint can be configured on the :ref:`api_pim_partitionqospolicy`
data member of the :ref:`dds_layer_publisher_publisherQos` or :ref:`dds_layer_subscriber_subscriberQos` objects.
This member holds a list of Partition name strings.
If no Partition is defined for an entity, it will be automatically included in the default nameless Partition.
Therefore, a Publisher and a Subscriber that specify no Partition will still
be able to communicate through the default nameless Partition.

.. warning::

    Partitions are linked to the endpoint and not to the changes.
    This means that the endpoint history is oblivious to modifications in the Partitions.
    For example, if a Publisher switches Partitions and afterwards needs to resend some older change again,
    it will deliver it to the new Partition set, regardless of which Partitions were defined
    when the change was created.
    This means that a late joiner Subscriber may receive changes that were created when another
    set of Partitions was active.

Wildcards in Partitions
-----------------------

.. _1003.2-1992 section B.6: https://standards.ieee.org/standard/1003_2-1992.html

Partition name entries can have wildcards following the naming conventions defined by the
POSIX ``fnmatch`` API (`1003.2-1992 section B.6`_).
Entries with wildcards can match several names, allowing an endpoint to easily be included in several Partitions.
Two Partition names with wildcards will match if either of them matches the other one according to ``fnmatch``.
That is, the matching is checked both ways.
For example, consider the following configuration:

- A Publisher with Partition ``part*``
- A Subscriber with Partition ``partition*``

Even though ``partition*`` does not match ``part*``, these Publisher and Subscriber
will communicate between them because ``part*`` matches ``partition*``.

Note that a Partition with name ``*`` will match any other partition **except the default Partition**.

Full example
------------

Given a system with the following Partition configuration:

+----------------+---------+--------------------------------+
| Participant_1  | Pub_11  | {"Partition_1", "Partition_2"} |
+                +---------+--------------------------------+
|                | Pub_12  | {"*"}                          |
+----------------+---------+--------------------------------+
| Participant_2  | Pub_21  | {}                             |
+                +---------+--------------------------------+
|                | Pub_22  | {"Partition*"}                 |
+----------------+---------+--------------------------------+
| Participant_3  | Subs_31 | {"Partition_1"}                |
+                +---------+--------------------------------+
|                | Subs_32 | {"Partition_2"}                |
+                +---------+--------------------------------+
|                | Subs_33 | {"Partition_3"}                |
+                +---------+--------------------------------+
|                | Subs_34 | {}                             |
+----------------+---------+--------------------------------+

The endpoints will finally match the Partitions depicted on the following table.
Note that ``Pub_12`` does not match the default Partition.

+--------------+-------------------+-------------------+---------------------------------------+
|              | Participant_1     | Participant_2     | Participant_3                         |
|              +---------+---------+---------+---------+---------+---------+---------+---------+
|              | Pub_11  | Pub_12  | Pub_21  | Pub_22  | Subs_31 | Subs_32 | Subs_33 | Subs_34 |
+--------------+---------+---------+---------+---------+---------+---------+---------+---------+
| Partition_1  |    ✓    |    ✓    |    ✕    |    ✓    |    ✓    |    ✕    |    ✕    |    ✕    |
+--------------+---------+---------+---------+---------+---------+---------+---------+---------+
| Partition_2  |    ✓    |    ✓    |    ✕    |    ✓    |    ✕    |    ✓    |    ✕    |    ✕    |
+--------------+---------+---------+---------+---------+---------+---------+---------+---------+
| Partition_3  |    ✕    |    ✓    |    ✕    |    ✓    |    ✕    |    ✕    |    ✓    |    ✕    |
+--------------+---------+---------+---------+---------+---------+---------+---------+---------+
| {default}    |    ✕    |    ✕    |    ✓    |    ✕    |    ✕    |    ✕    |    ✕    |    ✓    |
+--------------+---------+---------+---------+---------+---------+---------+---------+---------+

The following table provides the communication matrix for the given example:

+--------------------------+-------------------+-------------------+
|                          | Participant_1     | Participant_2     |
|                          +---------+---------+---------+---------+
|                          | Pub_11  | Pub_12  | Pub_21  | Pub_22  |
+----------------+---------+---------+---------+---------+---------+
| Participant_3  | Subs_31 |    ✓    |    ✓    |    ✕    |    ✓    |
+                +---------+---------+---------+---------+---------+
|                | Subs_32 |    ✓    |    ✓    |    ✕    |    ✓    |
+                +---------+---------+---------+---------+---------+
|                | Subs_33 |    ✕    |    ✓    |    ✕    |    ✓    |
+                +---------+---------+---------+---------+---------+
|                | Subs_34 |    ✕    |    ✕    |    ✓    |    ✕    |
+----------------+---------+---------+---------+---------+---------+

The following piece of code shows the set of parameters needed for the use case depicted in this example.


.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF-QOS-PARTITIONS
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->CONF-QOS-PARTITIONS
        :end-before: <!--><-->
        :lines: 2-3,5-
        :append: </profiles>


