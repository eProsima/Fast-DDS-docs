Version 1.6.0
=============

This release includes the following features:

* :ref:`persistence`
* Security access control plugin API and builtín :ref:`access-permissions` plugin.

Also bug fixing.

**Note:** If you are upgrading from an older version than 1.4.0, it is advisable to regenerate generated source from IDL files using
*fastrtpsgen*

Previous versions
-----------------

Version 1.5.0
^^^^^^^^^^^^^

This release includes the following features:

* Configuration of Fast RTPS entities through XML profiles.
* Added heartbeat piggyback support.

Also bug fixing.

**Note:** If you are upgrading from an older version than 1.4.0, it is advisable to regenerate generated source from IDL files using
*fastrtpsgen*

Version 1.4.0
^^^^^^^^^^^^^

This release includes the following:

* Added secure communications.
* Removed all Boost dependencies. Fast RTPS is not using Boost libraries anymore.
* Added compatibility with Android.
* Bug fixing.

**Note:** After upgrading to this release, it is advisable to regenerate generated source from IDL files using
*fastrtpsgen*

Version 1.3.1
^^^^^^^^^^^^^

This release includes the following:

* New examples that illustrate how to tweak Fast RTPS towards different applications.
* Improved support for embedded Linux.
* Bug fixing.

Version 1.3.0
^^^^^^^^^^^^^

This release introduces several new features:

* Unbound Arrays support: Now you can send variable size data arrays. 
* Extended Fragmentation Configuration: It allows you to setup a Message/Fragment max size different to the standard 64Kb limit. 
* Improved logging system: Get even more introspection about the status of your communications system.
* Static Discovery: Use XML to map your network and keep discovery traffic to a minimum.
* Stability and performance improvements: A new iteration of our built-in performance tests will make benchmarking easier for you.
* ReadTheDocs Support: We improved our documentation format and now our installation and user manuals are available online on ReadTheDocs.

Version 1.2.0
^^^^^^^^^^^^^

This release introduces two important new features:

* Flow Controllers: A mechanism to control how you use the available bandwidth avoiding data bursts. The controllers allow you to specify the maximum amount of data to be sent in a specific period of time. This is very useful when you are sending large messages requiring fragmentation.
* Discovery Listeners: Now the user can subscribe to the discovery information to know the entities present in the network (Topics, Publishers & Subscribers) dynamically without prior knowledge of the system. This enables the creation of generic tools to inspect your system.

But there is more:

* Full ROS2 Support: Fast RTPS is used by ROS2, the upcoming release of the Robot Operating System (ROS).
* Better documentation: More content and examples.
* Improved performance.
* Bug fixing.

