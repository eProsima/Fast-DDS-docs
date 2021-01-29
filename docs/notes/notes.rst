.. _release_notes:

Version 2.2.0
=============

This minor release is API compatible with the previous minor release, but introduces **ABI breaks** on
two of the three public APIs:

* Methods and attributes have been added on several classes of the DDS-PIM high-level API, so indexes of
  symbols on dynamic libraries may have changed.

* Methods and attributes have been added on several classes of the RTPS low-level API, so indexes of
  symbols on dynamic libraries may have changed.

* Old Fast-RTPS high-level API remains ABI compatible.

This release adds the following **features**:

* Data Sharing delivery (avoids transport encapsulation for localhost communications)
* Complete DDS-PIM high-level API declarations
* Extension APIs allowing zero-copy delivery (both intra-process and inter-process)
* Upgrade to Quality Level 1

It also includes the following **improvements**:

* Code coverage policy
* Added several tests to increase coverage
* Increased GUID uniqueness
* Allow logInfo messages to be compiled on build types other than debug

Some important **bugfixes** are also included:

* Fixed timed events manager race condition
* Fixed payload protection issues with SHM transport
* Writers correctly handle infinite resource limits on keyed topics
* Fixed unsafe code on AESGCMGMAC plugin
* Several fixes for IPv6 (whitelists, address parser)
* Fixes on liveliness timing handling
* Fixed warnings building on C++20

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

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
