.. _release_notes:

Version 2.0.1
=============

This release includes the following bug fixes:

* Fixed sending GAPs to late joiners
* Fixed asserting liveliness on data reception
* Avoid calling :func:`OpenSSL_add_all_algorithms` when not required

Other improvements:

* Fixing warnings

PRs in merge order:
`#1295 <https://github.com/eProsima/Fast-DDS/pull/1295>`_,
`#1300 <https://github.com/eProsima/Fast-DDS/pull/1300>`_,
`#1304 <https://github.com/eProsima/Fast-DDS/pull/1304>`_,
`#1290 <https://github.com/eProsima/Fast-DDS/pull/1290>`_,
`#1307 <https://github.com/eProsima/Fast-DDS/pull/1307>`_.

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastrtpsgen*.
  If you are upgrading from a version older than 1.10.0, regenerating the code is *recommended*.


Previous versions
=================

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
