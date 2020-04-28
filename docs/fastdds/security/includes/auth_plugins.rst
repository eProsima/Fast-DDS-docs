Authentication plugins
----------------------

They provide authentication on the discovery of remote participants.
When a remote participant is detected, Fast RTPS tries to authenticate using the activated Authentication plugin.
If the authentication process finishes successfully then both participants match and discovery protocol continues.
On failure, the remote participant is rejected.

You can activate an Authentication plugin using Participant property ``dds.sec.auth.plugin``. Fast RTPS provides a
built-in Authentication plugin. More information on :ref:`auth-pki-dh`.