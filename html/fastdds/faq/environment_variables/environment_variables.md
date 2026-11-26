<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="freq-env-variables-questions"></a>

<a id="environment-variables-frequently-asked-questions"></a>

# Environment Variables Frequently Asked Questions

**What are the most important environment variables that affect the behavior of Fast DDS?**

<br/> `FASTDDS_DEFAULT_PROFILES_FILE`, `SKIP_DEFAULT_XML`, `FASTDDS_BUILTIN_TRANSPORTS`, `ROS_DISCOVERY_SERVER`, `ROS_SUPER_CLIENT`, `FASTDDS_STATISTICS`, `FASTDDS_ENVIRONMENT_FILE`.
For further information, refer to [Environment variables](../../env_vars/env_vars.md#env-vars).

---

**What is the purpose of the "FASTDDS_DEFAULT_PROFILES_FILE" environment variable?**

<br/> Defines the location of the default profile configuration XML file.
For further information, refer to [FASTDDS_DEFAULT_PROFILES_FILE](../../env_vars/env_vars.md#env-vars-fastdds-default-profiles-file).

---

**What happens when the variable "SKIP_DEFAULT_XML" is set to 1?**

<br/> Skips looking for a default profile configuration XML file.
If this variable is set to 1, Fast DDS will load the configuration parameters directly from the classes’ definitions without looking for the `DEFAULT_FASTDDS_PROFILES.xml` in the working directory.
For further information, refer to [SKIP_DEFAULT_XML](../../env_vars/env_vars.md#env-vars-skip-default-xml).

---

**What is the primary purpose of the "FASTDDS_BUILTIN_TRANSPORTS" environment variable?**

<br/> Setting this variable allows to modify the builtin transports that are initialized during the [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) creation.
For further information, refer to [FASTDDS_BUILTIN_TRANSPORTS](../../env_vars/env_vars.md#env-vars-builtin-transports).

---

**What is the purpose of the "ROS_DISCOVERY_SERVER" environment variable?**

<br/> Setting this variable configures the DomainParticipant to connect to one or more servers using the Discovery Server discovery mechanism.
For further information, refer to [ROS_DISCOVERY_SERVER](../../env_vars/env_vars.md#env-vars-ros-discovery-server).

---

**What happens to a "DomainParticipant" when its discovery protocol is set to "SIMPLE" and "ROS_SUPER_CLIENT" is set to TRUE?**

<br/> If the DomainParticipant’s discovery protocol is set to `SIMPLE`, and `ROS_SUPER_CLIENT` is set to `TRUE`, the participant is automatically promoted to a `SUPER_CLIENT`.
For further information, refer to [ROS_SUPER_CLIENT](../../env_vars/env_vars.md#env-vars-ros-super-client).

---

**What is the purpose of setting the "FASTDDS_STATISTICS" environment variable, according to the provided information?**

<br/> Setting this variable configures the [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) to enable the statistics DataWriters which topics are contained in the list set in this environment variable.
For further information, refer to [FASTDDS_STATISTICS](../../env_vars/env_vars.md#env-vars-fastdds-statistics).

---

**What happens when you set the "FASTDDS_ENVIRONMENT_FILE" environment variable to a JSON file?**

<br/> Setting this environment variable to an existing `json` file allows to load the environment variables from the file instead of from the environment.
For further information, refer to [FASTDDS_ENVIRONMENT_FILE](../../env_vars/env_vars.md#env-vars-fastdds-environment-file).

<br/>
