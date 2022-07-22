.. _release_notes:

Version 2.3.5
=============

This release includes the following **improvements**:

1. Fixed several flaky tests.
2. Use native inter-process on Windows.
3. Support for partitions on DataWriterQoS and DataReaderQoS.
4. Support for GCC 12.
5. Correctly implement delete_contained_entities.

This release also includes the following **bugfixes**:

1.Fixed deadline issue on volatile DataWriter.
2. Allow updating partitions to an empty set.
3. Fixed order of returned samples on topics with keys.
4. Fixed issues in LivelinessManager.
5. Correctly give priority to intra-process over data-sharing.
6. Avoid bad_node_size exception when cross-building.
7. Fixed build errors with OpenSSL 3.0.
8. Avoid a volatile data-sharing reader to block a writer.
9. Fixed history record issues with persistence.
10. Correctly disable DataReader on destruction.
11. Fixed various GCC 11 warnings.
12. Fixed payload pool handling on EDPSimple destructor.
13. Fixed read after free on security code.
14. Fixed null dereference on XML parser.
15. Ensure correct boost singleton destruction order.
16. Enable memory protection on DataSharing readers.
17. TCP reconnection issues.
18. MemberDescriptor fully qualified name.
19. Fix recommended statistics DataReaderQos to enable backwards compatibility.
20. Fixed dangling sample references with big data.
21. Fixed deadlocks and data races.
22. Fixed reconnection to Discovery Server.
23. Other minor fixes.

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

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
