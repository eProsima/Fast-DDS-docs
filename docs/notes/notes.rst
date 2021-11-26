.. _release_notes:

Version 2.4.1
=============

This release includes the following **improvements**:

1. Fixed several flaky tests
2. Improved bandwidth usage of GAPs and HEARTBEATs
3. Correctly implement delete_contained_entities
4. Use native inter-process on Windows
5. Improved performance of unregister_instance
6. Improved OSS-fuzz integration
7. Support for partitions on DataWriterQoS and DataReaderQoS
8. Some documentation improvements
9. Removed unused macro to avoid naming clashes

This release includes the following **bugfixes**:

1. Avoid bad_node_size exception when cross building
2. Fixed build on old compilers
3. Fixed buffers exhaustion when compiled with statistics
4. Fixed runtime addition of Discovery Servers
5. Fixed dangling sample references with big data
6. Fixed history record issues with persistence
7. Correctly disable DataReader on destruction
8. Fixed alignment issues on XTypes QoS policies serialization
9. Fixed reconnection to Discovery Server
10. Correctly use builtin publisher for statistics DataWriters
11. Fixed various GCC-11 warnings
12. Use only public APIs from foonathan::memory
13. Fixed installation directories for DDS examples
14. Fixed read after free on security code

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.4.0.rst
.. include:: previous_versions/v2.3.4.rst
.. include:: previous_versions/v2.3.3.rst
.. include:: previous_versions/v2.3.2.rst
.. include:: previous_versions/v2.3.1.rst
.. include:: previous_versions/v2.3.0.rst
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
