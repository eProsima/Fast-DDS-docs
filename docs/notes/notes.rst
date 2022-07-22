.. _release_notes:

Version 2.1.2
=============

This release includes the following **improvements**:

1. Allow fully qualified name in TypeDescriptor.
1. Use native inter-process on Windows.
1. Support for GCC 12.
1. Support of googletest using colcon.

This release also includes the following **bugfixes**:

1. Fixed recovery of shared memory buffers.
1. Fixed issues in LivelinessManager.
1. Fixed default multicast locators.
1. Fixed TCP issues.
1. Fixed deadlocks and data races.
1. Fixed deadline issue on volatile DataWriter.
1. Avoid bad_node_size exception when cross-building.
1. Fixed order of returned samples on topics with keys.
1. Allow updating partitions to an empty set.
1. Supress OpenSSL 3.0 warnings.
1. MemberDescriptor fully qualified name.
1. Fixed history record issues with persistence.
1. Fixed reconnection to Discovery Server.
1. Other minor fixes.

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from a version older than 1.10.0, regenerating the code is *recommended*.

Previous versions
=================

.. include:: previous_versions/v2.1.1.rst
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
