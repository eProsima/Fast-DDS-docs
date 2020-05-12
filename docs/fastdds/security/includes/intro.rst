In compliance with the `DDS Security <https://www.omg.org/spec/DDS-SECURITY/1.1/>`_ specification, Fast DDS provides
secure communication by implementing pluggable security at four levels.

* DomainParticipants authentication.
* Access control of Entities.
* Data encryption.

By default, Fast RTPS doesn't compile security support.
You can activate it adding ``-DSECURITY=ON`` at CMake configuration step.
For more information about Fast RTPS compilation, see :ref:`installation-from-sources`.

You can activate and configure security plugins through :class:`eprosima::fastrtps::Participant` attributes using
properties.
A :class:`eprosima::fastrtps::rtps::Property` is defined by its name (:class:`std::string`) and its value
(:class:`std::string`).
Throughout this page, there are tables showing you the properties used by each security plugin.

Built-in plugins
----------------

The current version comes out with three security built-in plugins:

* :ref:`auth-pki-dh`: this plugin provides authentication using a trusted *Certificate Authority* (CA).
* :ref:`access-permissions`: this plugin provides access control to Participants at the Domain and Topic level.
* :ref:`crypto-aes-gcm-gmac`: this plugin provides authenticated encryption using Advanced Encryption Standard (AES) in
  Galois Counter Mode (AES-GCM).

