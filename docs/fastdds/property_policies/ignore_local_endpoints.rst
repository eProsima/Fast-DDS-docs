.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _property_ignore_local_endpoints:

Ignore Local Endpoints
----------------------

By default, Fast DDS will automatically match all the endpoints (meaning |DataReaders| and |DataWriters|) belonging to a
given |DomainParticipant| as soon as they share the same |Topic| and have compatible Qos.
This however can result in undesired feedback whenever an application creates a |DataReader| and a |DataWriter| under
the same |DomainParticipant| on a shared |Topic|.
Although this feedback can be filtered out at the application level upon data reception by filtering out messages coming
from a |DataWriter| belonging to the same |DomainParticipant| on the |DataReader| receiving the data (by looking at the
|GuidPrefix_t-api|), this entails for a data sample to go all the way to the |DataReaderListener| just to be discarded
by an overcomplicated application business logic.
For this reason, Fast DDS offers the possibility of instructing the |DomainParticipant| to avoid the matching of local
endpoints through the following property:

.. list-table::
   :header-rows: 1
   :align: left

   * - PropertyPolicyQos name
     - PropertyPolicyQos value
     - Default value
   * - ``"fastdds.ignore_local_endpoints"``
     - ``"true"``/``"false"``
     - ``"false"``

.. tab-set::

    .. tab-item:: C++
       :sync: cpp

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: // IGNORE_LOCAL_ENDPOINTS_DOMAINPARTICIPANT
            :end-before: //!--
            :dedent: 8

    .. tab-item:: XML
       :sync: xml

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->IGNORE_LOCAL_ENDPOINTS_DOMAINPARTICIPANT<-->
            :end-before: <!--><-->
            :lines: 2-4,6-18,20-21

.. note::
    An invalid value of ``fastdds.ignore_local_endpoints`` results in the default behaviour.
