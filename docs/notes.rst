Version 1.8.0
=============

This release includes the following features:

* Implementation of IDL 4.2
* Implementation of :ref:`deadline-qos` QoS
* Implementation of :ref:`lifespan-qos` QoS
* Implementation of :ref:`disable-positive-acks-qos` QoS
* Secure sockets on TCP transport (:ref:`TLS`)

It also adds the following improvements and bug fixes:

* Real-time improvements: non-blocking write calls for best-effort writers, addition of fixed size strings, fixed size bitmaps, resource limited vectors, etc
* Duration parameters now use nanoseconds
* Configuration of participant mutation tries (see :ref:`participantconfiguration`)
* Automatic calculation of the port when a value of 0 is received on the endpoint custom locators
* Non-local addresses are now filtered from whitelists
* Optimization of check for acked status for stateful writers
* Linked libs are now not exposed when the target is a shared lib
* Limitation on the domain ID has been added
* Fix for non-deterministic tests
* Fix for ReaderProxy history being reloaded incorrectly in some cases
* Fix for RTPS domain hostid being potentially not unique
* Fix for participants with different lease expiration times failing to reconnect

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastrtpsgen*

Previous versions
-----------------

Version 1.7.2
^^^^^^^^^^^^^

This release fixes an important bug:

* Allocation limits on subscribers with a KEEP_LAST QoS was taken from resource limits configuration
  and didn't take history depth into account.

It also has the following improvements:

* Vendor FindThreads.cmake from CMake 3.14 release candidate to help with sanitizers.
* Fixed format of gradle file.

Some other minor bugs and performance improvements.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastrtpsgen*

Version 1.7.1
^^^^^^^^^^^^^

This release included the following features:

* LogFileConsumer added to the logging system
* Handle FASTRTPS_DEFAULT_PROFILES_FILE environment variable indicating the default profiles XML file
* XML parser made more restrictive and with better error messages

It also fixes some important bugs:
* Fixed discovery issues related to the selected network interfaces on Windows
* Improved discovery times
* Workaround ASIO issue with multicast on QNX systems
* Improved TCP transport performance
* Improved handling of key-only data submessages

Some other minor bugs and performance improvements.

**KNOWN ISSUES**

* Allocation limits on subscribers with a KEEP_LAST QoS is taken from resource limits configuration
  and doesn't take history depth into account.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastrtpsgen*

Version 1.7.0
^^^^^^^^^^^^^

This release included the following features:

* :ref:`comm-transports-tcp`
* :ref:`dynamic-types`
* Security 1.1 compliance

Also bug fixing, allocation and performance improvements.

**Note:** If you are upgrading from an older version, it is **required** to regenerate generated source from IDL files
using *fastrtpsgen*

Version 1.6.0
^^^^^^^^^^^^^

This release included the following features:

* :ref:`persistence`
* Security access control plugin API and builtin :ref:`access-permissions` plugin.

Also bug fixing.

**Note:** If you are upgrading from an older version than 1.4.0, it is advisable to regenerate generated source from IDL
files using *fastrtpsgen*

Version 1.5.0
^^^^^^^^^^^^^

This release included the following features:

* Configuration of Fast RTPS entities through XML profiles.
* Added heartbeat piggyback support.

Also bug fixing.

**Note:** If you are upgrading from an older version than 1.4.0, it is advisable to regenerate generated source from IDL
files using *fastrtpsgen*

Version 1.4.0
^^^^^^^^^^^^^

This release included the following:

* Added secure communications.
* Removed all Boost dependencies. Fast RTPS is not using Boost libraries anymore.
* Added compatibility with Android.
* Bug fixing.

**Note:** After upgrading to this release, it is advisable to regenerate generated source from IDL files using
*fastrtpsgen*

Version 1.3.1
^^^^^^^^^^^^^

This release included the following:

* New examples that illustrate how to tweak Fast RTPS towards different applications.
* Improved support for embedded Linux.
* Bug fixing.

Version 1.3.0
^^^^^^^^^^^^^

This release introduced several new features:

* Unbound Arrays support: Now you can send variable size data arrays.
* Extended Fragmentation Configuration: It allows you to setup a Message/Fragment max size different to the standard
  64Kb limit.
* Improved logging system: Get even more introspection about the status of your communications system.
* Static Discovery: Use XML to map your network and keep discovery traffic to a minimum.
* Stability and performance improvements: A new iteration of our built-in performance tests will make benchmarking
  easier for you.
* ReadTheDocs Support: We improved our documentation format and now our installation and user manuals are available
  online on ReadTheDocs.

Version 1.2.0
^^^^^^^^^^^^^

This release introduced two important new features:

* Flow Controllers: A mechanism to control how you use the available bandwidth avoiding data bursts.
  The controllers allow you to specify the maximum amount of data to be sent in a specific period of time.
  This is very useful when you are sending large messages requiring fragmentation.
* Discovery Listeners: Now the user can subscribe to the discovery information to know the entities present in the
  network (Topics, Publishers & Subscribers) dynamically without prior knowledge of the system.
  This enables the creation of generic tools to inspect your system.

But there is more:

* Full ROS2 Support: Fast RTPS is used by ROS2, the upcoming release of the Robot Operating System (ROS).
* Better documentation: More content and examples.
* Improved performance.
* Bug fixing.

