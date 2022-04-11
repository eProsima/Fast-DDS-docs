.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.6.0
=============

This minor release is API compatible with the previous minor release, but introduces **ABI breaks** on two of the three
public APIs:

* Methods and attributes have been added on several classes of the DDS-PIM high-level API, so indexes of symbols on
  dynamic libraries may have changed. Some API is also being deprecated.
* Methods and attributes have been added on several classes of the RTPS low-level API, so indexes of symbols on dynamic
  libraries may have changed.
* Old Fast-RTPS high-level API remains ABI compatible.

This minor release includes the following **features**:

1. :ref:`Allow modifying remote server locators at runtime <DS_modify_server_list>`
2. :ref:`Add statistics physical information to DATA[p] using properties <property_policies_physical_data>`
3. :ref:`Content filter discovery information RTPS API <contentfilterlimits>`
4. Endpoint discovery RTPS API
5. ``on_sample_lost`` RTPS API
6. Transport layer API extension
7. :ref:`XML support for Fast DDS CLI <cli_discovery_run>`
8. :ref:`New exchange format to reduce bandwidth in Static Discovery <property_policies_edp_exchange_format>`

It also includes the following **improvements**:

1. Support lowercase keywords on SQL filter
2. Separate initialization and enabling of BuiltinProtocols
3. Add ``disable_positive_acks`` to Static Discovery XML
4. Several updates in the DDS-PIM API
5. Support for octet vectors on XML parser
6. Update README and roadmap
7. Update Fast-CDR submodule to v1.0.24
8. Add new CMake option ``APPEND_PROJECT_NAME_TO_INCLUDEDIR``

Some **bugfixes** are also included:

1. Fix MatchedStatus ``last_*_handle``
2. Fix recommended statistics DataReaderQos to enable backwards compatibility
3. Fixes for supporting Python bindings in Windows platforms
4. Fix publishing physical data on statistics topic
5. Other minor fixes and improvements

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.5.1.rst
.. include:: previous_versions/v2.5.0.rst
.. include:: previous_versions/v2.4.2.rst
.. include:: previous_versions/v2.4.1.rst
.. include:: previous_versions/v2.4.0.rst
.. include:: previous_versions/v2.3.4.rst
.. include:: previous_versions/v2.3.3.rst
.. include:: previous_versions/v2.3.2.rst
.. include:: previous_versions/v2.3.1.rst
.. include:: previous_versions/v2.3.0.rst
.. include:: previous_versions/v2.2.1.rst
.. include:: previous_versions/v2.2.0.rst
.. include:: previous_versions/v2.1.1.rst
.. include:: previous_versions/v2.1.0.rst
.. include:: previous_versions/v2.0.3.rst
.. include:: previous_versions/v2.0.2.rst
.. include:: previous_versions/v2.0.1.rst
.. include:: previous_versions/v2.0.0.rst
.. include:: previous_versions/v1.10.1.rst
.. include:: previous_versions/v1.10.0.rst
.. include:: previous_versions/v1.9.5.rst
.. include:: previous_versions/v1.9.4.rst
.. include:: previous_versions/v1.9.3.rst
.. include:: previous_versions/v1.9.2.rst
.. include:: previous_versions/v1.9.1.rst
.. include:: previous_versions/v1.9.0.rst
.. include:: previous_versions/v1.8.5.rst
.. include:: previous_versions/v1.8.4.rst
.. include:: previous_versions/v1.8.3.rst
.. include:: previous_versions/v1.8.2.rst
.. include:: previous_versions/v1.8.1.rst
.. include:: previous_versions/v1.8.0.rst
.. include:: previous_versions/v1.7.3.rst
.. include:: previous_versions/v1.7.2.rst
.. include:: previous_versions/v1.7.1.rst
.. include:: previous_versions/v1.7.0.rst
.. include:: previous_versions/v1.6.0.rst
.. include:: previous_versions/v1.5.0.rst
.. include:: previous_versions/v1.4.0.rst
.. include:: previous_versions/v1.3.1.rst
.. include:: previous_versions/v1.3.0.rst
.. include:: previous_versions/v1.2.0.rst
