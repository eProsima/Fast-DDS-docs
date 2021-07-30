.. _release_notes:

Version 2.3.4
=============

This release includes the following **improvements**:

1. Support of googletest using colcon
2. Network latency reports source participant
3. Update Fast DDS Gen to v2.0.2

This release includes the following **bugfixes**:

1. Fix mutex lock count on :class:`PDPListener`
2. Limit :class:`SequenceNumberSet` number of bits on deserialization
3. Fix segmentation fault on discovery server
4. Fix deadlock with security and timers
5. Fix bug using not protected code in a test
6. Fix deadlock with :class:`LivelinessManager`
7. Fix interval loop on events
8. Fix run event when was cancelled
9. Validate sequence range on :class:`CDRMessage::readSequenceNumberSet`
10. Fix subscription throughput data generation
11. Allow examples to build on QNX
12. Fix code on SHM clean
13. Accept Statistics DataWriters in Discovery Server
14. Fix read/take behavior when a future change is found
15. Correctly handle deserialization errors on ``read_next_sample()`` / ``take_next_sample()``
16. Fixing :class:`SequenceNumberSet_t` deserialization
17. Proper history clean up when a reader unmatches a writer
18. Unprotected code loaning samples
19. Fix publication throughput statistic on volatile writers
20. Fix Fast DDS CLI server name
21. Several fixes in examples and tests

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

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
