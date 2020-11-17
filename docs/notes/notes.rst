.. _release_notes:

Version 2.1.0
=============

This minor release is API compatible with the previous minor release, but introduces **ABI breaks** on
two of the three public APIs:

* Methods and attributes have been added on several classes of the DDS-PIM high-level API, so indexes of
  symbols on dynamic libraries may have changed.

* Methods and attributes have been added on several classes of the RTPS low-level API, so indexes of
  symbols on dynamic libraries may have changed.

* Old Fast-RTPS high-level API remains ABI compatible.

Users of the RTPS low-level API should also be aware of the following **API deprecations**:

* History::reserve_Cache has been deprecated

  * Methods RTPSWriter::new_change or RTPSReader::reserveCache should be used instead

* History::release_Cache has been deprecated

  * Methods RTPSWriter::release_change or RTPSReader::releaseCache should be used instead

This release adds the following **features**:

* Support persistence for large data
* Added support for `on_requested_incompatible_qos` and `on_offered_incompatible_qos`
* SKIP_DEFAULT_XML environment variable
* Added FORCE value to THIRDPARTY cmake options
* New log consumer (StdOutErrConsumer)
* Added methods to get qos defined in XML Profile
* Support for persistence on TRANSIENT_LOCAL

It also includes the following **improvements**:

* Internal refactor for intra-process performance boost
* Allow usage of foonathan/memory library built without debug tool
* Large data support on performance tests
* Reduced flakiness of several tests

Some important **bugfixes** are also included:

* Fixed behavior of several DDS API methods
* Fixed interoperability issues with RTI connext
* Fixed DLL export of some methods
* Avoid redefinition of compiler defined macros
* Fixed some intra-process related segmentation faults and deadlocks
* Fixed large data payload protection issues on intra-process
* Fixed C++17 and VS 2019 warnings
* Fixed linker problems on some platforms
* Fixed transient local retransmission after participant drop
* Fixed assertion failure on persistent writers

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastrtpsgen*.
  If you are upgrading from a version older than 1.10.0, regenerating the code is *recommended*.

Previous versions
=================

.. include:: previous_versions/v2.0.2.rst
.. include:: previous_versions/v2.0.1.rst
.. include:: previous_versions/v2.0.0.rst
.. include:: previous_versions/v1.10.0.rst
.. include:: previous_versions/v1.9.4.rst
.. include:: previous_versions/v1.9.3.rst
.. include:: previous_versions/v1.9.2.rst
.. include:: previous_versions/v1.9.1.rst
.. include:: previous_versions/v1.9.0.rst
.. include:: previous_versions/v1.8.4.rst
.. include:: previous_versions/v1.8.3.rst
.. include:: previous_versions/v1.8.2.rst
.. include:: previous_versions/v1.8.1.rst
.. include:: previous_versions/v1.8.0.rst
.. include:: previous_versions/v1.7.2.rst
.. include:: previous_versions/v1.7.1.rst
.. include:: previous_versions/v1.7.0.rst
.. include:: previous_versions/v1.6.0.rst
.. include:: previous_versions/v1.5.0.rst
.. include:: previous_versions/v1.4.0.rst
.. include:: previous_versions/v1.3.1.rst
.. include:: previous_versions/v1.3.0.rst
.. include:: previous_versions/v1.2.0.rst
