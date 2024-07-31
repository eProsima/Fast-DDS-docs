.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _freq_persistence_service_questions:

PERSISTENCE SERVICE Frequently Asked Questions
==============================================


.. collapse::  What is persistence in eProsima Fast DDS?




    :Answer:

    **Persistence** is a mechanism that allows recovering a previous state on starting the DDS. This is done by configuring the DataWriter's history to be stored in a persistent database, so that the DataWriter can load its history from it on creation. Furthermore, DataReaders can be configured to store the last notified change in the database, so that they can recover their state on creation.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  Why is it useful?




    :Answer:

    It adds robustness to applications in case of unexpected shutdowns.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How is the persistence database managed?




    :Answer:

    A persistence plugin must be configured for managing the database using the property ``dds.persistence.plugin``.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What type of database does the persistence plugin use?




    :Answer:

    It uses SQLite3 API.

|
