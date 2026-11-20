<a id="docker-shm-in-docker"></a>

<a id="leveraging-fast-dds-shm-in-docker-deployments"></a>

# Leveraging Fast DDS SHM in Docker deployments

By default, *Fast DDS* enables both a [Shared Memory Transport](../fastdds/transport/shared_memory/shared_memory.md#transport-sharedmemory-sharedmemory) and [Data-sharing delivery](../fastdds/transport/datasharing.md#datasharing-delivery) (when
the configuration allows it, see [Constraints](../fastdds/transport/datasharing.md#datasharing-delivery-constraints)).
The way *Fast DDS* utilizes to find out whether a remote [DomainParticipant](../fastdds/dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) is running on the same host as the local
one is through a hashing of the network interfaces, which entails that two Docker container which are not sharing the
same network stack will be detected as two different hosts, and therefore *Fast DDS* will defer to communicating through
the [UDP Transport](../fastdds/transport/udp/udp.md#transport-udp-udp) instead.

To enable the use of both the *Shared Memory Transport* and *Data-sharing* in Docker deployments, two additional
[options](https://docs.docker.com/engine/reference/commandline/run/#options) need to be passed to the docker run
command, those are:

* [–network=host](https://docs.docker.com/engine/reference/run/#network-settings): This option shares the host’s
  network stack with the containers, and therefore *Fast DDS* will be able to identify them as the same host.
* [–ipc=host](https://docs.docker.com/engine/reference/run/#ipc-settings---ipc): This option shares the host’s shared
  memory mechanism with the containers.
  Without it, *Fast DDS* will identify both containers as the same host, but since they will have separate shared memory
  spaces, they will not be able to communicate with one another.
  The use of this option is the Docker recommended way of sharing shared memory between containers, in opposition of for
  instance sharing the /dev/shm volume in Linux machines.
  A more advanced user could set the flag to shared, thus sharing the shared memory mechanism of one of the container
  with the others.

Starting with Fast DDS **[Pro]** 3.7.0, the participant property `fastdds.shm.only_host_id_on_locators` shall also be set
to `true` to force the SHM transport to not consider the machine ID on the locators, since the machine ID would
probably be different for each container, and therefore the SHM transport would not be able to find the other
participants.
See [SHM transport locators format](../fastdds/property_policies/non_consolidated_qos.md#property-policies-shm-host-id-only)  for more information.

```bash
docker run -it --rm --network=host --ipc=host [OPTIONS] <docker-image>
```
