.. _release_notes:

Version 2.3.0
=============

This minor release is API compatible with the previous minor release, but introduces **ABI breaks** on
two of the three public APIs:

* Methods and attributes have been added on several classes of the DDS-PIM high-level API, so indexes of
  symbols on dynamic libraries may have changed.

* Methods and attributes have been added on several classes of the RTPS low-level API, so indexes of
  symbols on dynamic libraries may have changed.

* Old Fast-RTPS high-level API remains ABI compatible.

This release adds the following **features**:

* Unique network flows
* Discovery super-client
* Statistics module API
* New flow controller API
* Static discovery configuration from raw string
* Added reception timestamp to SampleInfo
* Exposing get_unread_count on DataReader

It also includes the following **improvements**:

* Data-sharing delivery internal refactor
* Additional metadata on persistence databases
* Refactor on ReturnCode_t to make it switch friendly
* Performance tests refactored to use DDS-PIM high-level API
* Receive const pointers on delete_xxx methods
* Discovery server improvements
* Made SOVERSION follow major.minor

Some important **bugfixes** are also included:

* Fixed shared memory usage on QNX
* Fixed reference counting on internal pools
* Fixed singleton destruction order
* Fixed interoperability issues with x-types information
* Fixed recovery of shared memory buffers
* Lifespan support in persistent writers

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.


Previous versions
=================

.. include:: previous_versions/v2.2.0.rst
.. include:: previous_versions/v2.1.0.rst
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
