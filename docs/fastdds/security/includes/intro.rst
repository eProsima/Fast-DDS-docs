Fast RTPS can be configured to provide secure communications. For this purpose, Fast RTPS implements pluggable security
at three levels: authentication of remote participants, access control of entities and encryption of data.

By default, Fast RTPS doesn't compile security support.
You can activate it adding ``-DSECURITY=ON`` at CMake configuration step.
For more information about Fast RTPS compilation, see :ref:`installation-from-sources`.

You can activate and configure security plugins through :class:`eprosima::fastrtps::Participant` attributes using
properties.
A :class:`eprosima::fastrtps::rtps::Property` is defined by its name (:class:`std::string`) and its value
(:class:`std::string`).
Throughout this page, there are tables showing you the properties used by each security plugin.
