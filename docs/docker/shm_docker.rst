.. include:: ../03-exports/aliases.include

.. _docker_shm_in_docker:

Leveraging Fast DDS SHM in Docker deployments
---------------------------------------------

By default, *Fast DDS* enables both a :ref:`transport_sharedMemory_sharedMemory` and :ref:`datasharing-delivery` (when
the configuration allows it, see :ref:`datasharing-delivery-constraints`).
The way *Fast DDS* utilizes to find out whether a remote |DomainParticipant| is running on the same host as the local
one is through a hashing of the network interfaces, which entails that two Docker container which are not sharing the
same network stack will be detected as two different host, and therefore *Fast DDS* will defer to communicating through
the :ref:`transport_udp_udp` instead.

To enable the use of both the Shared Memory Transport and Data-sharing in Docker deployments, two additional
`options <https://docs.docker.com/engine/reference/commandline/run/#options>`_ need to be passed to the `docker run`
command, those are:

* `--network=host <https://docs.docker.com/engine/reference/run/#network-settings>`_: This option shares the host's
  network stack with the containers, and therefore *Fast DDS* will be able to identify them as the same host.
* `--ipc=host <https://docs.docker.com/engine/reference/run/#ipc-settings---ipc>`_: This option shares the host's shared
  memory mechanism with the containers.
  Without it, *Fast DDS* will identify both containers as the same host, but since they will have separate shared memory
  spaces, they will not be able to communicate with one another.
  The use of this option is the Docker recommended way of sharing shared memory between containers, in opposition of for
  instance sharing the `/dev/shm` volume in Linux machines.
  A more advance user could set the flag to `shared`, thus sharing the shared memory mechanism of one of the container
  with the others.

.. code-block:: bash

   docker run -it --rm --network=host --ipc=host [OPTIONS] <docker-image>
