.. _release_notes:

Information about the release lifecycle can be found
`here <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_.

Version 2.6.11 (maintenance)
============================

.. important::
  According to our
  `release support guidelines <https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md>`_
  the v2.6 minor will only receive patches for critical issues and security fixes.

This release includes the following **critical fixes**:

#. Fix `CVE-2025-62599 <https://www.cve.org/CVERecord?id=CVE-2025-62599>`_
#. Fix `CVE-2025-62600 <https://www.cve.org/CVERecord?id=CVE-2025-62600>`_
#. Fix `CVE-2025-62601 <https://www.cve.org/CVERecord?id=CVE-2025-62601>`_
#. Fix `CVE-2025-62602 <https://www.cve.org/CVERecord?id=CVE-2025-62602>`_
#. Fix `CVE-2025-62603 <https://www.cve.org/CVERecord?id=CVE-2025-62603>`_
#. Fix `CVE-2025-64098 <https://www.cve.org/CVERecord?id=CVE-2025-64098>`_
#. Fix `CVE-2025-62799 <https://www.cve.org/CVERecord?id=CVE-2025-62799>`_
#. Fix `CVE-2025-64438 <https://www.cve.org/CVERecord?id=CVE-2025-64438>`_
#. Fix `CVE-2025-65016 <https://www.cve.org/CVERecord?id=CVE-2025-65016>`_

This release includes the following **improvements**:

#. Verify Safe DDS signature
#. Improvements in message receiver
#. Regenerate types with Fast DDS Gen 2.1.4
#. Upgrade to Fast CDR v1.0.29

This release includes the following **ci management updates**:

#. Add ``uncrustify`` to Github CI
#. Remove deprecated windows 2019 from CI in favor of 2022
#. Upgrade to macOS Sequoia

.. note::
  If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
  files using *fastddsgen*.
  If you are upgrading from any older version, regenerating the code is *highly recommended*.

Previous versions
=================

.. include:: previous_versions/v2.6.10.rst
.. include:: previous_versions/v2.6.9.rst
.. include:: previous_versions/v2.6.8.rst
.. include:: previous_versions/v2.6.7.rst
.. include:: previous_versions/v2.6.6.rst
.. include:: previous_versions/v2.6.5.rst
.. include:: previous_versions/v2.6.4.rst
.. include:: previous_versions/v2.6.3.rst
.. include:: previous_versions/v2.6.2.rst
.. include:: previous_versions/v2.6.1.rst
.. include:: previous_versions/v2.6.0.rst
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
