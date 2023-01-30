.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _library_settings:

Intra-process delivery profiles
-------------------------------

This section defines the XML elements available for configuring the :ref:`intraprocess-delivery` settings parameters in *Fast DDS*.
These elements are defined within the XML tag ``<library_settings>``.


Intra-process delivery configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Intra-process delivery configuration is performed through the XML elements listed in the following table.

.. list-table::
   :header-rows: 1
   :align: left

   * - Name
     - Description
     - Values
     - Default
   * - ``<intraprocess_delivery>``
     - Speed up communications between entities within the same process by avoiding any of the overhead involved in the transport layer.
     - ``OFF`` |br|
       ``USER_DATA_ONLY`` |br|
       ``FULL``
     - ``OFF``

**Example**

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->CONF-LIBRARY-SETTINGS<-->
    :end-before: <!--><-->