.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. role:: raw-html(raw)
    :format: html

.. _xtypes_extensions:

XTypes Extensions
-----------------

This section explain those QoS Policy extensions defined in the `XTypes Specification <https://www.omg.org/spec/DDS-XTypes/>`_:

.. contents::
    :local:
    :backlinks: none
    :depth: 1

.. _datarepresentationqospolicy:

DataRepresentationQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^^^

This XTypes QoS Policy states which data representations will be used by the |DataWriters| and |DataReaders|.

The DataWriters offer a single data representation that will be used to communicate with the matched DataReaders.
The DataReaders can request one or more data representations and in order to have communication with the DataWriter,
the offered data representation needs to be contained within the DataReader request.
See |DataRepresentationQosPolicy-api|.


List of QoS Policy data members:

+-----------------------------------------------------------+------------------------------------------+---------------+
| Data Member Name                                          | Type                                     | Default Value |
+===========================================================+==========================================+===============+
| |DataRepresentationQosPolicy::m_value-api|                | std::vector<:ref:`datarepresentationid`> | Empty vector  |
+-----------------------------------------------------------+------------------------------------------+---------------+

.. note::
     This QoS Policy concerns to Topic, DataReader and DataWriter entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _datarepresentationid:

DataRepresentationId
""""""""""""""""""""

There are three possible values (see |DataRepresentationId-api|):

* |XCDR_DATA_REPRESENTATION-api|:
  This option corresponds to the first version of the `Extended CDR Representation` encoding.
* |XML_DATA_REPRESENTATION-api|:
  This option corresponds to the `XML Data Representation`.
* |XCDR2_DATA_REPRESENTATION-api|:
  This option corresponds to the second version of the `Extended CDR Representation` encoding.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_DATA_REPRESENTATION_QOS
   :end-before: //!

XML
***
This QoS Policy cannot be configured using XML for the moment.

.. _typeconsistencyenforcementqospolicy:

TypeConsistencyEnforcementQosPolicy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::
    This QoS Policy will be implemented in future releases.

This XTypes QoS Policy extension defines the rules for determining whether the data type used in the
|DataWriter| is consistent with the one used in the |DataReader|.
See |TypeConsistencyEnforcementQosPolicy-api|.

List of QoS Policy data members:

.. list-table::
   :header-rows: 1

   * - Data Member Name
     - Type
     - Default Value
   * - |TypeConsistencyEnforcementQosPolicy::m_kind-api|
     - :ref:`typeconsistencykind`
     - |ALLOW_TYPE_COERCION-api|
   * - |TypeConsistencyEnforcementQosPolicy::m_ignore_sequence_bounds-api|
     - ``bool``
     - ``true``
   * - |TypeConsistencyEnforcementQosPolicy::m_ignore_string_bounds-api|
     - ``bool``
     - ``true``
   * - |TypeConsistencyEnforcementQosPolicy::m_ignore_member_names-api|
     - ``bool``
     - ``true``
   * - |TypeConsistencyEnforcementQosPolicy::m_prevent_type_widening-api|
     - ``bool``
     - ``true``
   * - |TypeConsistencyEnforcementQosPolicy::m_force_type_validation-api|
     - ``bool``
     - ``true``

* |TypeConsistencyEnforcementQosPolicy::m_kind-api|:
  It determines whether the type in the DataWriter type must be equal to the type in the DataReader or not.
  See :ref:`typeconsistencykind` for further details.
* |TypeConsistencyEnforcementQosPolicy::m_ignore_sequence_bounds-api|:
  This data member controls whether the sequence bounds are taken into account for type
  assignability or not. If its value is true, the sequences maximum lengths are not considered, which means that a
  sequence T2 with length L2 would be assignable to a sequence T1 with length L1, even if L2 is greater than L1.
  But if it is false, L1 must be higher or equal to L2 to consider the sequences as assignable.
* |TypeConsistencyEnforcementQosPolicy::m_ignore_string_bounds-api|:
  It controls whether the string bounds are considered for type assignation or not.
  If its value is true, the strings maximum lengths are not considered, which means that a string S2 with length L2
  would be assignable to a string S1 with length L1, even if L2 is greater than L1.
  But if it is false, L1 must be higher or equal to L2 to consider the strings as assignable.
* |TypeConsistencyEnforcementQosPolicy::m_ignore_member_names-api|:
  This boolean controls whether the member names are taken into consideration for
  type assignability or not.
  If it is true, apart from the member ID, the member names are considered as part of assignability, which means that
  the members with the same ID must also have the same name. But if the value is false, the member names are ignored.
* |TypeConsistencyEnforcementQosPolicy::m_prevent_type_widening-api|:
  This data member controls whether the type widening is allowed or not.
  If it is false, the type widening is permitted, but if true, a wider type cannot be assignable to a narrower type.
* |TypeConsistencyEnforcementQosPolicy::m_force_type_validation-api|:
  It controls if the service needs the type information to complete the matching between a
  DataWriter and a DataReader.
  If it is enabled, it must have the Complete Type Information, otherwise it is not necessary.

.. note::
     This QoS Policy concerns to DataReader entities.
     :raw-html:`<br />`
     It cannot be changed on enabled entities.

.. _typeconsistencykind:

TypeConsistencyKind
"""""""""""""""""""

There are two possible values:

* |DISALLOW_TYPE_COERCION-api|:
  The DataWriter and the DataReader must support the same data type in order to
  communicate.
* |ALLOW_TYPE_COERCION-api|:
  The DataWriter and the DataReader do not need to support the same data type in order to
  communicate as long as the DataReader's type is assignable from the DataWriter's type.

Example
"""""""

C++
***
.. literalinclude:: ../../../../../code/DDSCodeTester.cpp
   :language: c++
   :dedent: 8
   :start-after: //DDS_CHANGE_TYPE_CONSISTENCY_ENFORCEMENT_QOS
   :end-before: //!

XML
***
This QoS Policy cannot be configured using XML for the moment.
