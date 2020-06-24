.. _release_notes:

Version 2.0.0
=============

This release has the following **API breaks**:

* eClock API, which was deprecated on v1.9.1, has been removed
* `eprosima::fastrtps::rtps::RTPSDomain::createParticipant` methods now have an additional first argument `domain_id`
* Data member `domainId` has been removed from `eprosima::fastrtps::rtps::RTPSParticipantAttributes` and added to
  `eprosima::fastrtps::ParticipantAttributes`

Users should also be aware of the following **deprecation announcement**:

* All classes inside the namespace `eprosima::fastrtps` should be considered deprecated.
  Equivalent functionality is offered through namespace `eprosima::fastdds`.
* Namespaces beneath `eprosima::fastrtps` are not included in this deprecation, i.e.
  `eprosima::fastrtps::rtps` can still be used)

This release adds the following **features**:

* Added support for register/unregister/dispose instance
* Added DDS compliant API. This new API exposes all the functionality of the Publisher-Subscriber Fast RTPS API
  adhering to the `Data Distribution Service (DDS) version 1.4 specification <https://www.omg.org/spec/DDS/1.4>`_
* Added Security Logging Plugin (contributed by Cannonical Ltd.)
* Bump to FastCDR v1.0.14

It also includes the following bug fixes and improvements:

* Support for OpenSSL 1.1.1d and higher
* Support for latest versions of gtest
* Support for FreeBSD
* Fault tolerance improvements to Shared Memory transport
* Fixed segfault when no network interfaces are detected
* Correctly ignoring length of `PID_SENTINEL` on parameter list
* Improved traffic on PDP simple mode
* Reduced CPU and memory usage

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastrtpsgen*.
If you are upgrading from a version older than 1.10.0, regenerating the code is *recommended*.


Previous versions
=================

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
