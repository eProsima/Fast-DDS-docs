.. _examplexml:

Example
-------

In this section, there is a full XML example with all possible configuration.
It can be used as a quick reference, but it may not be valid due to incompatibility or exclusive properties.
Don't take it as a working example.


.. AFTER PUBLISHER->DURABILITY
     <durabilityService>
        <service_cleanup_delay>
            <seconds>10</seconds>
            <nanosec>0</nanosec>
        </service_cleanup_delay>
        <history_kind>KEEP_LAST</history_kind>
        <history_depth>20</history_depth>
        <max_samples>10</max_samples>
        <max_instances>2</max_instances>
        <max_samples_per_instance>10</max_samples_per_instance>
    </durabilityService>

.. AFTER SUBSCRIBER->DURABILITY
    <durabilityService>
        <service_cleanup_delay>
            <!-- DURATION -->
        </service_cleanup_delay>
        <history_kind>KEEP_LAST</history_kind>
        <history_depth>50</history_depth>
        <max_samples>20</max_samples>
        <max_instances>3</max_instances>
        <max_samples_per_instance>5</max_samples_per_instance>
    </durabilityService>

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-EXAMPLE<-->
    :end-before: <!--><-->
    :lines: 2,4-
