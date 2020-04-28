.. _dds_layer_security_access_control_plugins:

Access control plugins
----------------------

They provide validation of entities' permissions.
After a remote participant is authenticated, its permissions need to be validated and enforced.

Access rights that each entity has over a resource are described.
Main entity is the Participant and it is used to access or produce information on a Domain;
hence the Participant has to be allowed to run in a certain Domain.
Also, a Participant is responsible for creating Publishers and Subscribers that communicate over a certain Topic.
Hence, a Participant has to have the permissions needed to create a Topic, to publish
through its Publishers certain Topics, and to subscribe via its Subscribers to certain Topics.
Access control plugin can configure the Cryptographic plugin because its usage is based on the Participant's
permissions.

You can activate an Access control plugin using Participant property ``dds.sec.access.plugin``.
Fast RTPS provides a built-in Access control plugin.
More information on :ref:`access-permissions`.