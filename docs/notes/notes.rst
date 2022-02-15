.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.5.0
=============

This minor release is API compatible with the previous minor release, but introduces **ABI breaks** on
two of the three public APIs:

* Methods and attributes have been added on several classes of the DDS-PIM high-level API, so indexes of
  symbols on dynamic libraries may have changed.

* Methods and attributes have been added on several classes of the RTPS low-level API, so indexes of
  symbols on dynamic libraries may have changed.

* Old Fast-RTPS high-level API remains ABI compatible.

This minor release includes the following **features**:

1. :ref:`Support for PKCS#11 <pkcs11-support>` format URIs for private keys
2. Added interfaces for content filter APIs
3. Allow new :ref:`network interfaces to be detected at runtime <dynamic-network-interfaces>`
4. New API on :ref:`DataWriter <api_pim_datawriter>` to wait for a specific instance to be acknowledged
5. Added interfaces for :ref:`concatenation of transports <transport_transportApi_chaining>`
6. Allow :ref:`XML profiles to be loaded from a string <loadingapplyingprofiles>`
7. Allow :ref:`disabling piggyback heartbeat <xml_disableheartbeatpiggyback>` on XML and DataWriter QoS
8. New basic configuration example

It also includes the following **improvements**:

1. Working implementation of instance_state and view_state
2. Allow zero-valued keys
3. Made some type aliases public to ease python bindings integration
4. Improved performance by avoiding unnecessary payload copies for samples that are going to be rejected
5. Removed unnecessary headers from Log module public headers
6. Add support for Key annotation in TypeObjectFactory
7. Only export public symbols on non-windows platforms
8. Some documentation improvements

Some important **bugfixes** are also included:

1. Fixed payload pool handling on EDPSimple destructor
2. Fixed null dereference on XML parser
3. Correctly export XTypes related methods on Windows
4. Ensure correct boost singleton destruction order
5. Avoid warning when environment file filename is empty
6. Correctly set GUID of DataWriter and DataReader upon creation

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.4.1.rst
.. include:: previous_versions/v2.4.0.rst
.. include:: previous_versions/v2.3.4.rst
.. include:: previous_versions/v2.3.3.rst
.. include:: previous_versions/v2.3.2.rst
.. include:: previous_versions/v2.3.1.rst
.. include:: previous_versions/v2.3.0.rst
.. include:: previous_versions/v2.2.1.rst
.. include:: previous_versions/v2.2.0.rst
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
