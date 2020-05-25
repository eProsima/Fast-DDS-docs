.. include:: includes/aliases.rst

.. _librarySettingsAttributes:

Library settings
----------------

This section is devoted to general Fast DDS settings that are not constraint to specific |Entities|
(|DomainParticipants|, |DataWriters|, |DataReaders|) or functionality (:ref:`Trasports <comm-transports-configuration>`
and :ref:`DataTypes <dds_layer_definition_data_types>`).
All of them are gathered under the ``library_settings`` profile.
Currently only the :ref:`intraprocess-delivery` feature is comprised here.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CONF-LIBRARY-SETTINGS<-->
    :end-before: <!--><-->
    :dedent: 4
