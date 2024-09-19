.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _freq_persistence_service_questions:

PERSISTENCE SERVICE Frequently Asked Questions
==============================================

.. collapse::  What is persistence in eProsima Fast DDS?

    |br|

    Persistence is a mechanism that allows recovering a previous state on starting the DDS. This is done by configuring the DataWriter's history to be stored in a persistent database, so that the DataWriter can load its history from it on creation. Furthermore, DataReaders can be configured to store the last notified change in the database, so that they can recover their state on creation. For more information, see :ref:`persistence_service`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  Why is it useful?

    |br|

    It adds robustness to applications in case of unexpected shutdowns. For further information, see :ref:`persistence_service`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How is the persistence database managed?

    |br|

    A persistence plugin must be configured for managing the database using the property ``dds.persistence.plugin``. For more information, see :ref:`persistence_service_conf`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What type of database does the persistence plugin use?

    |br|

    It uses SQLite3 API. For more information, see :ref:`persistence_sqlite3_builtin_plugin`.

|
