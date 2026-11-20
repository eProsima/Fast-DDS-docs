<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="eol-versions"></a>

# EOL versions

<a id="fast-dds-pro"></a>

## Fast DDS **[Pro]**

<a id="version-3-5-eol"></a>

### Version 3.5 (EOL)

<a id="version-3-5-0-1-pro"></a>

#### Version 3.5.0.1 **[Pro]**

This release includes the following **improvements**:

1. Finer-grained flow controller scheduling
2. Regenerate types with Fast DDS Gen Pro v4.3.0.1
3. Congestion control (see [Congestion Control](../../fastdds/use_cases/large_data/large_data.md#congestion-control) )

<a id="fast-dds"></a>

## Fast DDS

<a id="id1"></a>

### Version 3.5 (EOL)

<a id="id2"></a>

#### [Version 3.5.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v3.5.0/index.html)

This release includes the following **improvements**:

1. Update Qt version to 6.8.3 in CI
2. Verify Safe DDS signature
3. Open `RTPSParticipantImpl` for extension
4. Use new sample rejection value
5. Make `DomainParticipant::get_instance_handle` return by value
6. Discovery Server clients always behave as super clients
7. Enhance filtering logic for virtual and external endpoints
8. Refactor Secure Discovery Server
9. Update `foonathan.net` links
10. Apply `uncrustify` to all source files
11. Remove unused `keep_alive` logic
12. RPC refactor
13. Bump version to 3.5.0
14. Extend `DomainParticipant` API to include return codes
15. Regenerate types with Fast DDS Gen 4.3.0
16. Update CMake minimum required and Tier1

This release includes the following **fixes**:

1. Fix compilation on PDPTests
2. Force history QoS consistency and change resource limits default values
3. Fix wrong update of default locators
4. Fix ABI compliance

This release includes the following **CI improvements**:

1. Branchout 3.5.x
2. Updates after v3.3.1 EOL
3. Run `xtypes` test cases in parallel

<a id="version-3-4-eol"></a>

### Version 3.4 (EOL)

<a id="id3"></a>

#### [Version 3.4.3 (EOL)](https://fast-dds.docs.eprosima.com/en/v3.4.3/index.html)

This release includes the following **critical fixes**:

1. Fix [CVE-2026-22591](https://www.cve.org/CVERecord?id=CVE-2026-22591)
2. Fix [CVE-2026-45094](https://www.cve.org/CVERecord?id=CVE-2026-45094)
3. Fix [CVE-2026-45095](https://www.cve.org/CVERecord?id=CVE-2026-45095)
4. Fix [CVE-2026-45096](https://www.cve.org/CVERecord?id=CVE-2026-45096)
5. Fix [CVE-2025-63829](https://www.cve.org/CVERecord?id=CVE-2025-63829)
6. Fix [CVE-2025-65865](https://www.cve.org/CVERecord?id=CVE-2025-65865)
7. Fix [CVE-2026-45093](https://www.cve.org/CVERecord?id=CVE-2026-45093)
8. Fix [CVE-2026-45092](https://www.cve.org/CVERecord?id=CVE-2026-45092)
9. Fix [CVE-2026-45097](https://www.cve.org/CVERecord?id=CVE-2026-45097)

This release includes the following **fixes**:

1. Fix `RTPSParticipantAttributes` internal data races
2. Fix some null-dereference errors
3. Fix wrong update of default locators
4. Make `on_xxx_matched` thread-safe

This release includes the following **improvements**:

1. Update foonathan.net links
2. Remove deprecated doxygen warnings
3. Updates after v3.3.1 EOL

This release includes the following **ci management updates**:

1. Run `xtypes` test cases in parallel
2. Improve windows tests

<a id="id4"></a>

#### [Version 3.4.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v3.4.2/index.html)

This patch release includes the following **security fixes**:

1. [CVE-2025-65016](https://github.com/eProsima/Fast-DDS/commit/65d17fa1c6f3fd4e6d46e01aaaa1bce4a5926cbe)
2. [CVE-2026-22590](https://github.com/eProsima/Fast-DDS/commit/0695732f91aeaf77c433a38a73cfb03e78e8a320)

This patch release includes the following **improvements**:

1. Improve some tests
2. Replace `logWarning` with `EPROSIMA_LOG_WARNING` in `RTCPHeader.h`
3. Improvements in message receiver
4. Improvements on compute_key generated code
5. Add new `StatefulWriterListener`
6. Remove out the responsibility for bandwidth limitation from `RTPSMessageGroup`
7. Update `RELEASE_SUPPORT.md` after `2.6.11` release
8. Updates on `3.4.x` after branchout of `3.5.x`
9. Verify Safe DDS signature
10. Improve Open SSL includes
11. Allow empty payloads in dispose/unregister operations
12. Reset irrelevant sequence numbers interval in proxy readers
13. Update fastcdr to `2.3.5`
14. Bump version to `3.4.2`

This patch release includes the following **fixes**:

1. Fix compilation errors against android-clang
2. Fix GAP messages are not sent when there is no Reader requesting the DATA
3. Fix handling of InstanceHandle use cases
4. Fix usage of invalid iterator in `DynamicDataImpl::deserialize()`
5. Fix build in GCC 15
6. Account for notification sample in `DataReaderHistory::get_first_untaken_info`
7. Fix DataReader history enforcement to respect `max_samples_per_instance`

This patch release includes the following **CI improvements**:

1. Single line for `ctest_args` in Mac CI
2. Free disk space in Ubuntu runners
3. Upgrade Linters CI runner image to `ubuntu-24.04`

<a id="id5"></a>

#### [Version 3.4.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v3.4.1/index.html)

This patch release includes the following **security fixes**:

* [CVE-2025-62599](https://www.cve.org/CVERecord?id=CVE-2025-62599)
* [CVE-2025-62600](https://www.cve.org/CVERecord?id=CVE-2025-62600)
* [CVE-2025-62601](https://www.cve.org/CVERecord?id=CVE-2025-62601)
* [CVE-2025-62602](https://www.cve.org/CVERecord?id=CVE-2025-62602)
* [CVE-2025-62603](https://www.cve.org/CVERecord?id=CVE-2025-62603)
* [CVE-2025-64098](https://www.cve.org/CVERecord?id=CVE-2025-64098)
* [CVE-2025-62799](https://www.cve.org/CVERecord?id=CVE-2025-62799)
* [CVE-2025-64438](https://www.cve.org/CVERecord?id=CVE-2025-64438)

This patch release includes the following **improvements**:

* Send heartbeat when [`RTPSWriter::wait_for_all_acked`](../../fastdds/api_reference/rtps/writer/RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriter18wait_for_all_ackedERKN3dds10Duration_tE) is called
* Update fastcdr submodule
* Update RELEASE_SUPPORT with release 3.2.3
* Apply `Uncrustify`

This patch release includes the following **fixes**:

* Remove residual generated files
* Guard `gnu::optimize` with `defined(__GNUC__)`
* Making system libraries private in `target_link_libraries`
* Remove manually change of `/Zi` MSVC option
* Avoid `PROJECT_SOURCE_DIR` and `make_unique` when building blackbox tests
* Avoid casting `GUID_t` into `InstanceHandle_t`
* Fixed some data races

It also includes the following **CI management improvements**:

* Allow CI to run tests with manual dispatch
* Reduce disk usage in TSAN job
* Upgrade to MacOS Sequoia

#### IMPORTANT
When upgrading to version 3.4.1 it is **highly recommended** to regenerate generated source from IDL files
using at least [Fast DDS-Gen v4.2.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.2.0).
But it is advisable to regenerate them using the latest patch version of `Fast DDS-Gen v4.2.x`.

<a id="id6"></a>

#### [Version 3.4.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v3.4.0/index.html)

This minor release includes the following **features**:

* *Fast DDS Pro* features:
  : * Automatic detection of changes in IP addresses and network interfaces ([IP mobility](../../fastdds/use_cases/ip_mobility/ip_mobility.md#ip-mobility))
    * Support for [DDS-TSN](../../fastdds/use_cases/tsn/tsn.md#use-case-dds-tsn)
    * [Low Bandwidth Transports](../../fastdds/transport/low-bandwidth-transports.md#low-bandwidth-transports)
* Pass value of TransportPriorityQosPolicy to transport layer
* Add field `original_writer_info` to `WriteParams` and `SampleInfo`
* Support annotated types and builtin annotations in IDL Parser
* Iterate over declared types processed with IDL Parser
* Support aliases in [Content Filtered Topic](../../fastdds/dds_layer/topic/contentFilteredTopic/contentFilteredTopic.md#dds-layer-topic-contentfilteredtopic)

This minor release includes the following **improvements**:

* [Linux binaries](../../installation/binaries/binaries_linux.md#linux-binaries) installation with `.deb` packages
* Update to Fast DDS Gen 4.2.0
* Return sample notifying changes on instance state
* Performance improvements with large history caches
* Fix doxygen documentation typos
* Change `eprosima::fastdds::rtps::octet` to be `uint8_t` according to C++11 standard
* Add `tcp_negotiation_timeout` to XML Schema
* Some improvements on python generated code

This minor release includes the following **fixes**:

* Fix memory consumption issues in PKIDH plugin
* Fix tests build with strict C++11
* Fix socket buffer size handling
* Avoid setting thread affinity of 0 in Mac
* Some RPC related fixes
* Fix latency & throughput tests names
* Allow empty partition list to match against `"*"`
* Fix duplicated transport when using specific XML with CLI
* Fix topic argument in Discovery Server example
* Fix type code generation for IDL files with no structs
* Some TCP transport fixes
* Remote writers considered volatile by default

This minor release includes the following **CI management updates**:

* Add `--quiet` to `git submodule` invocations in CI
* Split Mac CI build and testing phases
* Update fallback branches to use environment variables

#### IMPORTANT
When upgrading to version 3.4.0 it is **highly recommended** to regenerate generated source from IDL files
using at least [Fast DDS-Gen v4.2.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.2.0).
But it is advisable to regenerate them using the latest patch version of `Fast DDS-Gen v4.2.x`.

<a id="version-3-3-eol"></a>

### Version 3.3 (EOL)

<a id="id8"></a>

#### [Version 3.3.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v3.3.1/index.html)

This patch release includes the following **critical fixes**:

1. Fix [CVE-2025-62599](https://www.cve.org/CVERecord?id=CVE-2025-62599)
2. Fix [CVE-2025-62600](https://www.cve.org/CVERecord?id=CVE-2025-62600)
3. Fix [CVE-2025-62601](https://www.cve.org/CVERecord?id=CVE-2025-62601)
4. Fix [CVE-2025-62602](https://www.cve.org/CVERecord?id=CVE-2025-62602)
5. Fix [CVE-2025-62603](https://www.cve.org/CVERecord?id=CVE-2025-62603)
6. Fix [CVE-2025-64098](https://www.cve.org/CVERecord?id=CVE-2025-64098)
7. Fix [CVE-2025-62799](https://www.cve.org/CVERecord?id=CVE-2025-62799)
8. Fix [CVE-2025-64438](https://www.cve.org/CVERecord?id=CVE-2025-64438)
9. Fix [CVE-2025-65016](https://www.cve.org/CVERecord?id=CVE-2025-65016)
10. Fix [CVE-2026-22590](https://www.cve.org/CVERecord?id=CVE-2026-22590)

This patch release includes the following **improvements**:

1. Update `3.3.x` to be a supported branch
2. Avoid setting thread affinity of 0 in Mac
3. Update Fast CDR version
4. Improve signal handling in RPC example
5. Change `non_blocking_send` threshold to `sendBufferSize`
6. Avoid sending duplicated `ACKs` in `DataSharing`
7. Use binary search in `find_change_nts`
8. Guard `gnu::optimize` with `defined(__GNUC__)`
9. Apply `uncrustify`
10. Do not export discovery server as `cmake` target
11. Send `heartbeat` when `wait_for_all_acked` is called
12. Update `RELEASE_SUPPORT` with release `3.2.3`
13. Protect `DataReaderInstance` alive_writers attribute
14. Avoid casting `GUID_t` into `InstanceHandle_t`
15. Support aliases in Content Filtered Topic
16. Allow empty partition list to match against `*`
17. Release change while authentication fails
18. Improve latency tests
19. Improvements in message receiver
20. Improvements on `compute_key` generated code
21. Protect `PDPStatelessWriter` attributes
22. Verify Safe DDS signature
23. Improve Open SSL includes
24. Regenerate types with Fast DDS Gen `v4.1.1`
25. Allow to send empty payloads in dispose/unregister operations
26. Reset irrelevant sequence numbers interval in proxy readers

This patch release includes the following **fixes**:

1. Fix socket buffer size handling
2. Fix latency & throughput tests names
3. Return sample notifying changes on instance state
4. Fix doxygen documentation for `iHandle2GUID`
5. Set different content filter signatures for each requester
6. Fix doxygen to avoid errors in python bindings
7. Fix topic argument in Discovery Server example
8. Add missing field to `xsd`
9. Fix duplicated transport when using specific XML with CLI
10. Remote writers considered volatile by default
11. Solve TCP disconnect while incomplete read deadlock
12. Remove residual generated files
13. Handle maximum deadline misses case
14. Improve some tests
15. Replace `logWarning` with `EPROSIMA_LOG_WARNING` in `RTCPHeader.h`
16. Avoid `PROJECT_SOURCE_DIR` and `make_unique` when building blackbox tests
17. Fix `GAP` messages are not sent when there is no Reader requesting the `DATA`
18. Fix usage of invalid iterator in `DynamicDataImpl::deserialize()`
19. Fix build in `GCC 15`

This patch release includes the following **ci management updates**:

1. Add `--quiet` to git submodule invocations
2. Update fallback branches to use environment variables
3. Split Mac CI build and testing phases
4. Add missing IDL to code generator script
5. Remove residual generated files
6. Allow CI to run tests with manual dispatch
7. Free disk space in CI runners
8. Upgrade to `macOS Sequoia`
9. Single line for `ctest_args` in Mac CI
10. Upgrade Linters CI runner image to `ubuntu-24.04`

#### IMPORTANT
When upgrading to version 3.3.1 it is **highly recommended** to regenerate generated source from IDL files
using at least [Fast DDS-Gen v4.1.1](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.1.1).
But it is advisable to regenerate them using the latest patch version of `Fast DDS-Gen v4.1.x`.

<a id="id17"></a>

#### [Version 3.3.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v3.3.0/index.html)

This minor release includes the following **features**:

1. RPC related:
   * Add generic interfaces for servers
   * Enhanced discovery feature
   * Add RPC over DDS example
   * Add methods for `TypeObject` registration of RPC types
2. QoS Policies added to `EDP` messages. New property to send `eProsima` Extensions QoS
3. Allow `ResourceLimitsQos` serialization method compatibility
4. Add `WireProtocolConfigQos` to optional QoS serialization and make `Monitor Service` always serialize them
5. `DataWriter` sample prefilter
6. Support modules in IDL Parser
7. New version of EDP static discovery
8. Process `key-only` payloads
9. Add `has_more_replies` to `WriteParams` and `SampleInfo`
10. `JSON` to `DynamicData` deserializer

This minor release includes the following **improvements**:

1. Improvements on RPC entities
2. Add regression test checking `type_information` is always copied in `ProxyData` copies
3. Add references to documentation
4. Update `ROSin` image links in `README`
5. Update reception timestamp when it is added to the instance
6. Updates due to `EOL` versions
7. Allow creation of built-in content filters with different type name
8. Add localhost capability only to `UDP` and `TCP` transports
9. Add `ParticipantBuiltinTopicData` header in `StatisticsDomainParticipant`
10. Fix DS initialization on `"any"` if no interfaces are available
11. Make `DomainParticipant` creation fail if `MaxMessageSize` is lower than `PDP` package size
12. Avoid calling `setIPv4` in `TCPTransportInterface`
13. Bump `asio` to `1.34.2`
14. Update types for Fast DDS Gen v4.1.0 Release

This minor release includes the following **fixes**:

1. Solve Discovery Server race conditions
2. Address some compilation warnings with `GCC` latest
3. Properly delete secure endpoints if registration fails
4. Add condition for `string_builder` `nullptr` on `XMLDynamicParser` in case bound is zero
5. Add multicast locator to correct locator list in `ReaderProxydata`
6. Avoid Data Race in `Reader Locator`
7. Fix `ParticipantGenericMessage` serialization
8. Fix build on `QNX`
9. Fix custom feature IDL files
10. Pass `Uncrustify` to all `ipp` files
11. Fix `null-dereference` error on Ubuntu

This minor release includes the following **ci management updates**:

1. Move `SimpleCommunication` and `system tests` in Windows nightly
2. Split Windows workflow in different test suites
3. Add compilation with `GCC` latest to `fastdds_alternative_builds` Ubuntu CI
4. Add `uncrustify` to Github CI
5. Increase the warning level for Ubuntu CI compilation
6. Include all example tests in Windows CI
7. Add `VS143 toolset` to Windows CI
8. Install Fix `OpenSSL` version in windows example testing
9. Set `Windows 10` as `Tier 1`

#### IMPORTANT
When upgrading to version 3.3.0 it is **highly recommended** to regenerate generated source from IDL files
using at least [Fast DDS-Gen v4.1.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.1.0).
But it is advisable to regenerate them using the latest patch version of `Fast DDS-Gen v4.1.x`.

<a id="version-3-1-eol"></a>

### Version 3.1 (EOL)

<a id="id18"></a>

#### [Version 3.1.3 (EOL)](https://fast-dds.docs.eprosima.com/en/v3.1.3/index.html)

This release includes the following **improvements**:

1. Support compiler `MSYS2-MinGW`
2. Improve `max_allocations` calculation on SHM transport
3. Filter interested readers on `PDP writer`
4. Update types with `Fast DDS Gen v4.0.4`
5. Refactor builtin writers and readers creation
6. Add modules support to `idl_serialize`
7. Improve DS routines

This release includes the following **fixes**:

1. Fix error handling logic in `try_setting_buffer_size`
2. Avoid redefinition warning for `OPENSSL_API_COMPAT`
3. Solve fuzz `XMLParser Null-dereference`
4. Remove repeated header include and Fix log
5. Fix Security tag in `DS CLI`
6. Fix assertion on `OutputTrafficManager`
7. Fix cleanup race condition with exclusive and shared lock files
8. Remove `SHM` option from discovery server example
9. Fix `several_writers_on_unack_sample_removed` flaky test
10. Avoid sending statistics msg with big messages and no fragmentation
11. Fix windows example tests in Debug and mitigate `rtps` example flakiness
12. Address some compilation warnings with GCC latest
13. Solve Discovery Server race conditions
14. Fix `oss-fuzz` regressions
15. Avoid Data Race in Reader Locator

This release includes the following **ci improvements**:

1. Get correct Fast CDR related branch in CI
2. Force Asio `Thirdparty` in MacOS CI

#### IMPORTANT
When upgrading to version 3.1.3 it is **highly recommended** to regenerate generated source from IDL files
using at least [Fast DDS-Gen v4.0.3](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.0.3).
But it is advisable to regenerate them using the latest patch version of `Fast DDS-Gen v4.0.x`.

<a id="id19"></a>

#### [Version 3.1.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v3.1.2/index.html)

This release includes the following **fixes**:

1. Arithmetic overflow in fragment size calculations
2. Fix EDP reliability timings
3. Address `parseXMLMemberDynamicType` regressions
4. Fix double-locking issue in `DataSharingListener`
5. Fix Discovery CLI Tool in Windows (No privileges)
6. Fix log category name macro collision in `MacOS`
7. Handle socket buffer size setting when system’s maximum exceeded
8. Decouple transport receivers creation using unique network flows
9. Fix dropped samples on reliable writers
10. System Log error: infinite loop when setting thread affinity fails

This release includes the following **improvements**:

1. Benchmark example updated
2. Update sqlite from 3.36.0 to 3.47.2
3. Update submodules when cloning Fast DDS on CI
4. Improve PDPClients initialization
5. Log any errors before `cancel_init()`
6. Add `replace` tag to `package.xml`

#### IMPORTANT
When upgrading to version 3.1.2 it is **highly recommended** to regenerate generated source from IDL files
using at least [Fast DDS-Gen v4.0.3](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.0.3).
But it is advisable to regenerate them using the latest patch version of `Fast DDS-Gen v4.0.x`.

<a id="id21"></a>

#### [Version 3.1.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v3.1.1/index.html)

This patch release includes the following **features** in an ABI compatible manner:

1. New property to select preferred key agreement algorithm

This release includes the following **fixes**:

1. Fix reach of maximum buffers in `asio::send_to`
2. Fix TCP discovery server locators translation
3. Fix unique network flows with TCP transports
4. Address OSS-Fuzz\` regressions in XML parsers
5. Fix `DataReaderHistory` regression
6. Fix destruction data-race on participant removal in intra-process
7. Several fixes in security plugins and `SecurityManager`
8. Fix `cmake` generator evaluation
9. Fix build with `-Werror=template-id-cdtor`
10. Fix potential deadlock between `StatefulWriter` and `FlowController`

This release includes the following **improvements**:

1. Update commercial support section in README
2. Rename `RTPSParticipantImpl.h` to `RTPSParticipantImpl.hpp`
3. Check if `SHM` transport is disabled in `LARGE_DATA` modes
4. Improve Blackbox TCP tests suite
5. Test examples on Windows CI
6. Regenerate types with Fast DDS Gen 4.0.3

#### IMPORTANT
When upgrading to version 3.1.1 it is **highly recommended** to regenerate generated source from IDL files
using at least [Fast DDS-Gen v4.0.3](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.0.3).
But it is advisable to regenerate them using the latest patch version of `Fast DDS-Gen v4.0.x`.

<a id="id23"></a>

#### [Version 3.1.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v3.1.0/index.html)

This minor release includes the following **features** in an ABI compatible manner:

1. Allow running CI on external contributions
2. Allow for all durability configurations
3. Complete support for `Dynamic Network Interfaces`
4. Add [QoS getters from raw XML](../../fastdds/xml_configuration/making_xml_profiles.md#loading-raw-xml-profiles)

This release includes the following **improvements**:

1. Be less strict with parameter lengths
2. Modify help command of `XML CLI tool`
3. Register and propagate `MonitorService` status type object
4. Add tests for `TypeLookup service`
5. Convert `Fast DDS v2` to `v3` Migration Changes into a [Step-by-Step User Guide](../migration_guide.md#migration-guide)
6. Update `asio` version from `1.18.1` to `1.31.0`
7. Update `nlohmann/json` thirdparty version from `3.9.1` to `3.11.3`
8. Regenerate types with latest `Fast DDS-Gen v4.0.2`
9. Update `fastcdr` submodule with latest release
10. Update `roadmap`, `release support` and `versions` for `v3.1.0` release

This release includes the following **fixes**:

1. Update request reply example `README`
2. Fix `xtypes example` segfault if XML environment var is not set
3. Remove unused `validMatching` methods in `EDP`
4. Add Regression test for `Fast DDS-Gen` Use different `typename` for collections of `TK_UINT8` and `TK_BYTE`
5. Update latest `2.10.x` to `2.10.5`
6. Discard changes with big `key-only` payload and no key hash
7. Add test for support of enumeration literal `@value` annotation
8. Fix memory leak deleting `TypeLookupManager`
9. Fix `DynamicData` union deserialization when no member is selected
10. Split nightly jobs to one per workflow
11. Add references to new nightly jobs per supported branch in `README`
12. Change `xsd` files installation directory to `share/fastdds` and remove icons on `windows uninstall`
13. Fix `flow_controllers` comparison in `DomainParticipantQos` equality operator
14. Fix `CreateTopicWithDifferentTypeName_negative` flaky test
15. Warn instead of fail when `dds.communication.dynamic_interfaces` cannot be built
16. Fix wrong Data type referred in `flow control` example `README`

#### IMPORTANT
When upgrading to version 3.1.0 it is **required** to regenerate generated source from IDL files
using at least [Fast DDS-Gen v4.0.2](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.0.2).
But it is advisable to regenerate them using the latest patch version of `Fast DDS-Gen v4.0.x`.

<a id="version-3-0-eol"></a>

### Version 3.0 (EOL)

<a id="id24"></a>

#### [Version 3.0.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v3.0.2/index.html)

This release includes the following **improvements**:

1. Add support for `DLL` export on Windows
2. Modify help command of XML CLI tool
3. Register and propagate `MonitorService` status type object
4. Add tests for `TypeLookup` service
5. Discard changes with big `key-only` payload and no `key hash`
6. Improve on Step-by-Step User Guide from the Migration Guide
7. Add test for support of enumeration literal
8. Update new nightly jobs on `README`
9. Change `xsd` files installation directory to `share/fastdds` and remove icons on Windows uninstall
10. Update commercial support section in `README`
11. Rename `RTPSParticipantImpl.h` to `RTPSParticipantImpl.hpp`
12. Add new property to select preferred key agreement algorithm
13. Improve `OpenSSL` lifecycle handling
14. Improve Blackbox TCP tests suite
15. Improve `PDPClientsp` initialization
16. Update `sqlite` from 3.36.0 to 3.47.2
17. Improve debugging process by adding logs before running `SecurityManager::cancel_init()`
18. Update Benchmark example
19. Add `replace` tag to `package.xml`
20. Decouple transport receivers creation using unique network flows
21. Support compiler MSYS2-MinGW
22. Avoid redefinition warning for `OPENSSL_API_COMPAT`

Github CI management:

1. Allow running CI on external contributions
2. Add Windows CI example testing
3. Update submodules when cloning Fast DDS on CI
4. Add `flow_control`, `rtps`, `custom_payload_pool` and `content_filter` to Windows example CI testing

This release includes the following **fixes**:

1. Fix CVE-2025-24807
2. Update request reply refactored example `README`
3. Fix `xtypes` example issue if XML environment variable was not set
4. Fix length checks in the builtin decoder
5. Remove unused `validMatching` methods in `EDP`
6. Regression test for `Fast DDS-Gen`
7. Fix memory leak issue deleting `TypeLookupManager`
8. Fix issues in Dynamic Network Interfaces
9. Fix wrong Data type referred in flow control example
10. Address `oss fuzz` `XMLParser` regression
11. Fix destruction `data-race` on participant removal in `intra-process`
12. Fix `DataReaderHistory` regression
13. Fix `FASTDDS_SHM_TRANSPORT_DISABLED` for `LARGE_DATA` modes
14. Address `XMLDynamicParser` regression
15. Fix `SecurityManager` assertion in `Secure DS`
16. Fix issue on `participant_stateless`
17. Fix comparison in `is_update_allowed`
18. Fix `-Werror=template-id-cdtor`
19. Fix TCP discovery server locators translation
20. Fix `cmake` generator evaluation
21. Fix `TSAN` potential deadlock by lock inversion
22. Fix issue on `NetworkBuffers`
23. Fix unique network flows with TCP transports
24. Fix algorithm strings on `PermissionsToken` and `IdentityToken`
25. Arithmetic overflow in fragment size calculations
26. Address `parseXMLMemberDynamicType` regression
27. Fix `EDP` reliability timings
28. Fix null-dereference in `parseXMLMemberDynamicType`
29. Fix double-locking issue in `DataSharingListener`
30. Fix Discovery CLI Tool in Windows (No privileges)
31. Fix socket buffer size configuration mechanism
32. Fix issue on `reliable` `volatile` when all history acked
33. Fix log category name macro collision in `MacOS`
34. Fix issue of infinite loop when setting thread affinity fails
35. Fix error handling logic in `try_setting_buffer_size`
36. Fix call to `on_unacknowledged_sample_removed` when keep all history is used

#### IMPORTANT
When upgrading to version 3.0.2 it is **required** to regenerate generated source from IDL files
using at least [Fast DDS-Gen v4.0.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.0.0).
But it is advisable to regenerate them using the latest patch version of `Fast DDS-Gen v4.0.x`.

<a id="id25"></a>

#### [Version 3.0.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v3.0.1/index.html)

This patch release includes the following **improvements**:

1. Set string arguments as `const` references
2. Improve resilience against clock adjustments
3. Make Fast DDS build compatible with `GCC 9`
4. Use eProsima-CI action to install `Qt`
5. Update Fast CDR submodule to `v2.2.4`
6. Regenerate types with `Fast DDS Gen v4.0.1`

This release includes the following **fixes**:

1. Secure simple participants with `initialpeers` over TCP match
2. Register also the minimal created from the received complete `TypeObject` in the `TypeLookupService`
3. Fix issue with exclusive ownership and unordered samples
4. Fix compilation error in `Ubuntu 24.04`
5. Correct iterator increment after erasing elements in `connected_servers_list`
6. Fix request reply example isolated build

#### IMPORTANT
When upgrading to version 3.0.1 it is **required** to regenerate generated source from IDL files
using at least [Fast DDS-Gen v4.0.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.0.0).
But it is advisable to regenerate them using the latest patch version of `Fast DDS-Gen v4.0.x`.

<a id="id27"></a>

#### [Version 3.0.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v3.0.0/index.html)

Fast DDS v3.0.0 is a mayor release that entails some **API breaks** and new features.
This version is **not backwards compatible** with previous versions.
Refer to the [migration guide](https://github.com/eProsima/Fast-DDS/blob/master/UPGRADING.md) for hints moving to Fast DDS v3.0.0.

This release includes the following **API breaks**:

1. Fastrtps package name migration to fastdds.
2. API and namespace migration from RTPS to DDS.
3. Make private previously public API.
4. Make all public headers .hpp.
5. [XTypes](../../fastdds/xtypes/xtypes.md#dynamic-types) refactor.
6. [IPayloadPool](../../fastdds/rtps_layer/rtps_layer.md#rtps-layer-ipayload-pool-interface) refactor.
7. [Participant discovery structures](../../fastdds/api_reference/rtps/builtin/data/ParticipantBuiltinTopicData.md#api-rtps-builtin-data-participantbuiltintopicdata) refactor.
8. GUIDLess [Discovery Server](../../fastdds/discovery/discovery_server.md#discovery-server).
9. Gather-send implementation.
10. `const` qualify all data related inputs in DataWriter APIs.
11. Refactor RTPS reader APIs.
12. Refactor RTPS writer APIs.
13. Refactor RTPS `WriterHistory`.
14. Refactor `TopicDataType`.
15. Make `StdoutErrConsumer` default `LogConsumer`.
16. All `DataWriter::write` overloads return `ReturnCode_t` type.
17. Add [XML configuration](../../fastdds/use_cases/large_data/large_data.md#flow-controllers) for `FlowControllerDescriptor` and remove `ThroughputController`.
18. Move `DataReader::TypeConsistencyEnforcement` and `DataReader::DataRepresentation` from `TypeConsistency` to `DataReaderQos`.
19. Migrate `BuiltinEndpoints` defines to variables.
20. Remove `string_convert` header and source.
21. [Examples](https://github.com/eProsima/Fast-DDS/blob/master/examples/cpp/hello_world/README.md) refactor.
22. Update Fast DDS docs QoS examples.
23. Link SHM locator kind with Fast DDS major version.
24. Discard local SHM locators that cannot be opened.

This release includes the following new **features**:

1. [Type Propagation](../../fastdds/property_policies/non_consolidated_qos.md#property-type-propagation) policy.
2. [Create Participant with default profile](../../fastdds/dds_layer/domain/domainParticipant/createDomainParticipant.md#dds-layer-domainparticipant-creation-default-profile)
   (use environment XML configuration).
3. Create [DomainParticipantExtendedQos](../../fastdds/dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantextendedqos) class.
4. Add product version on Participant Discovery information.
5. Automatically unmatch remote participants on participant deletion.
6. Refactor Statistics module IDL.
7. Change PDP discovery database backup restore order.
8. Create `InitialConnection` for TCP initial peers.
9. New [Maximum Message Size](../../fastdds/property_policies/non_consolidated_qos.md#property-max-message-size) property to limit output datagrams size.
10. Use `PID_DOMAIN_ID` during PDP.
11. Ubuntu example testing automation infrastructure.
12. Update versions in fastdds.repos file.

This release includes the following **improvements**:

1. Make `DataReader::return_loan` return `RETCODE_OK` on loanable sequences without loans.
2. Use absolute paths when loading XML files.
3. Remove `FASTCDR_VERSION_MAJOR == 1` related code.
4. Setting `vendor_id` on received `CacheChange_t`.
5. Builtin data related improvements.
6. GitHub repository management.
7. [Migration guide](https://github.com/eProsima/Fast-DDS/blob/master/UPGRADING.md).
8. Update fastcdr thirdparty.
9. Documentation updates due to major version change.

This release includes the following **fixes**:

1. Handle errors when setting socket buffer sizes.
2. Do not require `PYTHON_VERSION` to be defined in .bat files.
3. Use `%*` instead of `loop` in .bat scripts.
4. Set DataSharing in `WriterProxyData` and `ReaderProxyData`.
5. Minor fix in `DDSBlackboxTestsBasic.cpp`.
6. Fix PDP and EDP unittest compilation.
7. Fix `AllocTest` compilation.
8. Correct liveliness state in a multiple reader and one writer scenario.
9. Only apply content filter to ALIVE changes.
10. Fix compilation when building static library.
11. Properly delete builtin statistics writers upon `delete_contained_entities()`.
12. Correctly initialize `MatchingFailureMask` constants to be used with the `std::bitset` API.
13. Change monitor service writer entity id.
14. Fix EDP build warning without security.
15. Fix compilation warning.
16. Fix discovery server not connecting due to ports logic.
17. Fix assertion in `TopicPayloadPool::release_history`.
18. Fix topic interference on `liveliness_changed` status.
19. Fix typo in CMakeLists.txt.
20. Solve `SecurityManager` memory issue.
21. Fuzzer regressions and fix build issues.
22. Address build issues for documentation and different platforms.
23. Add unsigned specification to literals.

#### IMPORTANT
When upgrading to version 3.0.0 it is **required** to regenerate generated source from IDL files
using [Fast DDS-Gen v4.0.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.0.0).

<a id="version-2-13-eol"></a>

### Version 2.13 (EOL)

<a id="id29"></a>

#### [Version 2.13.6 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.13.6/index.html)

This release includes the following **improvements**:

1. Create InitialConnection for TCP initial peers
2. Only apply content filter to ALIVE changes
3. Update version support documentation
4. Add XML configuration for `FlowControllerDescriptor`
5. Github CI improvements

This release includes the following **fixes**:

1. Fix DS servers not connecting due to ports logic
2. Correctly initialize `MatchingFailureMask` constants to be used with the `std::bitset` API
3. Fix topic interference on `liveliness_changed` status
4. Properly delete builtin statistics writers upon `delete_contained_entities()`
5. Fix Latency tests
6. Remove doxygen warnings

#### NOTE
When upgrading to version 2.13.6 it is **advisable** to regenerate generated source from IDL files
using [Fast DDS-Gen v3.2.1](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.2.1).

<a id="id30"></a>

#### [Version 2.13.5 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.13.5/index.html)

This release includes the following **features** in an ABI compatible manner:

1. New [Maximum Message Size](../../fastdds/property_policies/non_consolidated_qos.md#property-max-message-size) property to limit output datagrams size

This release includes the following **improvements**:

1. Improve ThreadSettingsQoS logging
2. Allow processing of AckNack submessages with `count == 0`
3. Internal refactor on port handling
4. Do not require `PYTHON_VERSION` to be defined in .bat files
5. Use `%*` instead of loop in `.bat` scripts
6. Consider library behavior changes as ABI breaks in the PR template checklist
7. Refactor IStatusQueryable and make monitor service interfaces private
8. Automatically unmatch remote participants on participant deletion
9. Handle errors when setting socket buffer sizes
10. Github CI management:
    > 1. Refactor Github CI sanitizer related jobs
    > 2. Build Fast DDS Python bindings in Fast DDS Docs Github CI job
    > 3. Build ShapesDemo on Ubuntu Github CI
    > 4. Fix Python Installation version in Github CI. Address failing system tests environment issues.
    > 5. Fix sanitizers CI test summary report
    > 6. Run selected VS tool on Windows CI
    > 7. Increase sleep to miss the deadline in macOS flaky tests
    > 8. Fix ShmTransport buffer recovery MacOS flaky test
    > 9. Set fallback branch for `get_related_branch_from_repo` correctly
    > 10. Add DNS entries to hosts files on github workflows

This release includes the following **fixes**:

1. Add check for XML API to PR template
2. Use absolute paths when loading XML files
3. Fix some leaks in XML DynamicTypes Parser
4. Force unlimited ResourceLimits if lower or equal to zero
5. Enforce SHM ports open mode exclusions
6. Run `is_plain` method with the corresponding data representation
7. Removed warning
8. Don’t require Fast CDR v2 in examples
9. Make reader `get_first_untaken_info()` coherent with `read()` / `take()`
10. Fix leak in `SecurityManager::participant_volatile_message_secure_writer_`
11. Fix CVE-2024-30258 / CVE-2024-30259
12. Fix support for `@key` annotation in Dynamic types
13. Set DataSharing in Writer|ReaderProxyData
14. Fix on_sample_lost notification on best-effort readers for fragmented samples
15. Correct liveliness state in a multiple reader - one writer scenario

#### NOTE
When upgrading to version 2.13.5 it is **advisable** to regenerate generated source from IDL files
using [Fast DDS-Gen v3.2.1](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.2.1).

<a id="id32"></a>

#### [Version 2.13.4 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.13.4/index.html)

This release includes the following **features** in an ABI compatible manner:

1. Expose [Authentication Handshake Properties](../../fastdds/property_policies/security.md#property-policies-security)

This release includes the following **improvements**:

1. Monitor service properly managing instances
2. Effectively assert `AUTOMATIC` / `MANUAL_BY_PARTICIPANT` liveliness
3. Add catch of out-of-range exception for thread settings port
4. TCP transport improvements:
   > 1. `TCPSendResources` cleanup
   > 2. TCP first message loss
   > 3. Set real TCP `non_blocking_send` limitation

This release includes the following **fixes**:

1. Fix hidden overloaded virtual methods
2. Fix Discovery Server over TCP using logical port
3. Protect asio exception fix
4. Fix flaky Log tests
5. Fix CVE-2024-28231
6. Add missing virtual destructor for StatisticsAncillary
7. Increase ack waiting time in `reliable_on_unack_sample_removed`
8. Fix versions in fastrtps.repos
9. GitHub CI fixes:
   > 1. Fix CI version management
   > 2. Add manual Ubuntu Github CI
   > 3. Avoid running GitHub CI if PR has conflicts
   > 4. Migrate apt package installation action to eProsima-CI
   > 5. Only run PRs CI when review requested
   > 6. Pin CMake version and `vm.mmap_rnd_bits` in sanitizer workflows
   > 7. Improve filtering of DNS tests

#### NOTE
When upgrading to version 2.13.4 it is **advisable** to regenerate generated source from IDL files
using [Fast DDS-Gen v3.2.1](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.2.1).

<a id="id34"></a>

#### [Version 2.13.3 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.13.3/index.html)

This patch release includes the following **improvements**:

1. Enabling multiple interfaces through whitelist in TCP servers
2. Set LARGE_DATA Participants logic with the same listening ports
3. Check History QoS inconsistencies

This patch release includes the following **fixes**:

1. Prevent index overflow and correctly assert the end iterator in DataSharing
2. Fix the shared memory cleaning script
3. Fix TCP reconnection after open logical port failure
4. Fix data race on PDP
5. Fix doxygen docs warnings. Prepare for compiling with Doxygen 1.10.0
6. Add missing TypeLookup listeners
7. Restore Blackbox tests names
8. Add macOS Github CI
9. Set 2.11.x as EOL

#### NOTE
When upgrading to version 2.13.3 it is **advisable** to regenerate generated source from IDL files
using [Fast DDS-Gen v3.2.1](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.2.1).

<a id="id36"></a>

#### [Version 2.13.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.13.2/index.html)

This patch release includes the following **fixes** and **improvements**:

1. Improve environment variable substitution algorithm
2. Add large data to the Advanced Configuration example
3. Discard already processed samples on `PDPListener`
4. Ignore `0x8007` if coming from other vendor
5. TCP unique client announced local port
6. TCP non-blocking send
7. TCP Client&Server Participant Decision-Making
8. Add non-throwing getters for socket info
9. Add a keyed fragmented change to the reader data instance only when it is completed
10. Include a variety of terminate process signals handlers in the discovery server
11. Make DataWriters always send the key hash on keyed topics
12. Update Fast DDS types with Fast DDS Gen to include `<cstdint>` in v1 types
13. Add serialization for Log::Kind to ostream
14. Fix wrong log info messages on TCP
15. Return const reference in `get_log_resources`
16. Remove unnecessary warning
17. Avoid a maybe-uninitialized warning
18. Add static cast to an unused variable
19. Prepare for v3.0.0 branch out
20. Windows CI fixes

#### NOTE
When upgrading to version 2.13.2 it is **advisable** to regenerate generated source from IDL files
using [Fast DDS-Gen v3.2.1](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.2.1).

<a id="id38"></a>

#### [Version 2.13.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.13.1/index.html)

#### NOTE
This release upgrades the following Fast DDS dependencies:

* [Fast CDR v2.1.3](https://github.com/eProsima/Fast-CDR/releases/tag/v2.1.3)
* [Fast DDS-Gen v3.2.1](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.2.1)

This release includes the following **improvements**:

1. Downgrade CMake minimum required version to 3.20.
2. Update PR template to include check for PR description, title and backports.
3. Update Fast CDR submodule to v2.1.3.

This patch release includes the following **fixes**:

1. Revert wrong fix for TCP deadlock on channel reuse.
2. Fix wrong history selection when removing PDP samples.
3. Fix data race when processing incoming PDP samples on different threads.
4. Fix DNS test filter in CMakeLists.
5. Fix deserialization of unions in generated code.

#### NOTE
When upgrading to version 2.13.1 it is **advisable** to regenerate generated source from IDL files
using [Fast DDS-Gen v3.2.1](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.2.1).

<a id="id41"></a>

#### [Version 2.13.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.13.0/index.html)

#### NOTE
This release upgrades the following Fast DDS dependencies:

* [Fast CDR v2.1.2](https://github.com/eProsima/Fast-CDR/releases/tag/v2.1.2)
* [Fast DDS-Gen v3.2.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.2.0)

This release includes the following **features**:

1. Support [Monitor Service](../../fastdds/statistics/monitor_service/monitor_service.md#monitor-service).
2. Enable configuration of [thread settings](../../fastdds/dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for all threads
   (both through the C++ API and XML configuration files).
3. Support Autofill port (automatic assignment of a port) for [TCP Transport](../../fastdds/transport/tcp/tcp.md#transport-tcp-tcp) listening port.
4. Support [TCP for Discovery Server](../../fastdds/use_cases/tcp/tcp_with_discovery_server.md#use-case-tcp-discovery-server) CLI and environment variable.
5. Usage of gtest_discover_tests.
6. Define a [super client](../../fastdds/env_vars/env_vars.md#env-vars-ros-super-client) by environment variable.
7. Support adding interfaces to the [interface whitelist](../../fastdds/transport/whitelist.md#whitelist-interfaces) by the name.
8. Add new methods to configure [Builtin Transport](../../fastdds/transport/tcp/tcp.md#transport-tcp-enabling).
9. Support DataRepresentationQos.
10. Change serialize function default behavior to omit the data representation.
11. Upgrade Fast CDR submodule to v2.1.2.
12. Update roadmap & platforms support.

This release includes the following **improvements**:

1. Rerun failed tests with ctest option instead of colcon’s.
2. Add CCache to all CI jobs.

This release includes the following **fixes**:

1. **Fast DDS bugfixes**
   > 1. Fix compilation of XMLProfileParserTests when building without security.
   > 2. Improve IgnoreNonExistentSegment test for Windows.
   > 3. Add missing thread includes.
   > 4. Fix warning in Mac rewarding unnecessary lambda capture.
   > 5. Use SO_EXCLUSIVEADDRUSE for Win32 unicast listening sockets.
   > 6. Fix gtest discovery timeout.
   > 7. Mark on_participant_discovery overload removal.
   > 8. Fix uninitialized member in BuiltinAttributes class.
   > 9. Fix set affinity directive for Android.
   > 10. Fix Monitor Service types & test without security.
   > 11. Fix TCP deadlock on channel reuse.
   > 12. Fix DNS filter in CMakeLists file for tests.
   > 13. Fix memory issues related to ciphering payload.
   > 14. Fix a bad-free when receiving a malformed DATA_FRAG submessage.
   > 15. Fix CVE-2023-50257.
   > 16. Fix compilation of Fast DDS Python tests.
   > 17. Fix data race on writer destruction while sending heartbeat.
   > 18. Fix build with TLS, when SECURITY=OFF and NO_TLS=OFF.
2. CI fixes:
   > 1. Fix colcon on github CI.
   > 2. Better handling of trigger events in docs CI.

#### NOTE
When upgrading to version 2.13.0 it is **advisable** to regenerate generated source from IDL files
using [Fast DDS-Gen v3.2.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.2.0).

<a id="version-2-12-eol"></a>

### Version 2.12 (EOL)

<a id="id43"></a>

#### [Version 2.12.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.12.2/index.html)

This release includes the following **features**:

1. Methods to configure [transport scenarios](../../fastdds/rtps_layer/rtps_layer.md#rtps-layer-builtin-transports)
2. Support `Autofill port` ([automatically set the port](../../fastdds/transport/tcp/tcp.md#transport-tcp-transportdescriptor)) for TCP Transport
3. Support [TCP for Discovery Server](../../fastdds/use_cases/tcp/tcp_with_discovery_server.md#use-case-tcp-discovery-server) CLI and environment variable
4. Define a [super client by environment variable](../../fastdds/env_vars/env_vars.md#env-vars-ros-super-client)
5. Change serialize function default behaviour to omit the data representation
6. `LARGE_DATA` Participants logic with same listening ports
7. TCP Client&Server Participant Decision-Making logic
8. Expose [Authentication Handshake Properties](../../fastdds/property_policies/security.md#property-policies-security)
9. Enabling multiple interfaces through whitelist in TCP servers
10. Add macOS and Ubuntu Github CI

This release includes the following **improvements**:

1. Improve environment variable substitution algorithm
2. Upgrade dependency version to last patch version in .repos file
3. Rerun failed tests with ctest option instead of colcon’s
4. Remove unnecessary TCP warning
5. Update PR template to include check for PR description, title and backports
6. Improvements in GitHub CI

This release includes the following **fixes**:

1. Fix TCP reconnection after open logical port failure
2. TCP unique client announced local port
3. TCP non-blocking send
4. Fix wrong log info messages on TCP
5. Improve `IgnoreNonExistentSegment` test
6. Use `SO_EXCLUSIVEADDRUSE` for Win32 unicast listening sockets
7. Fix DNS filter in CMakeLists file for tests
8. Fix bad-free when receiving malformed DATA_FRAG submessage
9. Fix memory problem related to ciphering payload
10. Fix CVE-2023-50257
11. Fix build with TLS, but not security
12. Fix comparison in `remove_from_pdp_reader_history`
13. Fix data race in `PDPListener` and `SecurityManager`
14. Discard already processed samples on `PDPListener`
15. Fix .repos versions
16. Fix the shared memory cleaning script
17. Fix data race on writer destruction while sending heartbeat
18. Return `const` reference to the shared pointer instead of a copy in `get_log_resources`
19. Ignore `0x8007` if coming from other vendor
20. Fix Doxygen docs warnings and prepare for compiling with Doxygen 1.10.0
21. Include variety of terminate process signals handler in discovery server
22. Add missing `TypeLookup` listeners
23. Add a keyed fragmented change to the reader data instance only when its completed
24. Fix data race on PDP
25. Check History QoS inconsistencies
26. Make DataWriters always send the key hash on keyed topics
27. Prevent index overflow and correctly assert the end iterator in DataSharing
28. Fix uninitialized member in `RTPSParticipantAttributes`
29. Remove unnecessary `std::move` in `FileWatch.hpp` causing warning
30. Add missing thread include
31. Add missing virtual destructor for `StatisticsAncillary`
32. Protect asio exception
33. `TCPSendResources` cleanup
34. Downgrade CMake version to 3.20

<a id="id44"></a>

#### [Version 2.12.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.12.1/index.html)

This release includes the following **improvements**:

1. Support for linking with Fast CDR v1.
2. The period for the timer within the [DisablePositiveACKsQosPolicy](../../fastdds/dds_layer/core/policy/eprosimaExtensions.md#disablepositiveacksqospolicy) is now updatable.
3. Log error message upon receiver resource creation failure.
4. CI and repository improvements.
5. Simplify code in CDRMessage.

This release includes the following **fixes**:

1. **Fast DDS bugfixes**
   > 1. Fix transient local durability for reliable readers using intra-process and data-sharing.
   > 2. Use STL implementation of Timed/RecursiveTimedMutex when MSVC >= 19.36.
   > 3. Fix updatability of immutable DataWriterQos.
   > 4. Fix the clang build for clang 14.
   > 5. Fix remote locators filtering when whitelist provided.
   > 6. Fix Data Race when updating liveliness changed in WLP.
   > 7. Add XML parser bit_bound bounds check.
   > 8. Fix missing mandatory attribute check in XML parser struct type.
   > 9. SHM transport: ignore non-existing segment on pop.
   > 10. Fix: mac address overflow on Windows.
2. CI fixes:
   > 1. Fix flow controllers unit tests compilation when using Fast CDR from thirdparty.
   > 2. PubSubAsReliable test fix.
   > 3. FileWatchTest fix for github windows CI.

#### NOTE
When upgrading to version 2.12.1 it is **advisable** to regenerate generated source from IDL files
using [Fast DDS-Gen v3.1.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.1.0).

<a id="id45"></a>

#### [Version 2.12.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.12.0/index.html)

#### NOTE
This release upgrades the following Fast DDS dependencies:

* [Fast CDR v2.0.0](https://github.com/eProsima/Fast-CDR/releases/tag/v2.0.0)
* [Fast DDS-Gen v3.0.1](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.0.1)

Please, read also the release notes of
[Fast DDS-Gen v3.0.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.0.0) to be aware of every possible
break in the application code.

As Fast DDS dependencies have been upgraded to new major releases, depending on the types defined in the IDL files,
it might be required to modify the user application source code besides recompiling it (more information can be found
in the corresponding release notes).

#### NOTE
There is a minor API break with previous v2.x versions: `MEMBER_INVALID` identifier was declared using `#define`.
In order to prevent polluting the user workspace, it has been transformed into a `constexpr` within
`eprosima::fastrtps::types` namespace.

This release includes the following **features**:

1. [New participant property](../../fastdds/property_policies/non_consolidated_qos.md#property-policies-shm-enforce-metatraffic) to configure SHM
   transport metatraffic behavior.
2. Exposed custom payload pool on DDS [DataWriter](../../fastdds/dds_layer/publisher/dataWriter/createDataWriter.md#dds-layer-publisher-datawriter-with-payload-pool-creation) and
   [DataReader](../../fastdds/dds_layer/subscriber/dataReader/createDataReader.md#dds-layer-subscriber-datareader-with-payload-pool-creation) declaration.
   > 1. Feature example.
3. [Processing environment variables in XML text](../../fastdds/xml_configuration/making_xml_profiles.md#xml-environment-variables).
4. Dependencies
   > 1. Upgrade internal type supports using latest Fast DDS-Gen release v3.0.0.
   >    This release introduces the following features:
   >    > 1. Support for @optional builtin annotation <optional_members>.
   >    > 2. Support for @extensibility builtin annotation <extensibility>.
   > 2. Upgrade Fast CDR submodule to v2.0.0 introducing XCDR encoding version 2.

This release includes the following **improvements**:

1. `fixed_string` comparison operators.
2. Remove mutex from TimedEventImpl (#3745, #3760)
3. Performance improvements on intraprocess and datasharing.
4. Improve Shared Memory resilience to crashing participants.
5. Improve scripts shebang portability.
6. Use `foonathan_memory` to reduce allocations in SharedMemManager.

This release includes the following **fixes**:

1. **Fast DDS bugfixes**
   : 1. Fixed XMLParser null-dereference when parsing log configuration.
     2. Allow participant XML profiles with no `<rtps>` tag.
     3. Fix encapsulation format in Writer Liveliness Protocol.
     4. Fix `DomainParticipant::register_remote_type`
        return when negotiating type.
     5. Fix strict real-time feature when using Flow Controller feature.
     6. Fix ParameterPropertyList increment operators.
     7. Fix bad-free when receiving malformed DATA submessage.
     8. Fix asymmetric whitelist matching.
     9. Fix heap-use-after-free on XMLElementParser.
     10. Fix History remove change return statement.
2. CI fixes
   : 1. Fix RemoteBuiltinEndpointHonoring blackbox test.
     2. Improve repository workflows.
     3. Use FASTRTPS_NO_LIB on unittest root folder.
     4. Fix Windows workflow.
3. Tools
   : 1. Remove C++11 check in `fastdds-discovery-server` CLI tool.
4. Examples
   : 1. Fix HelloWorldDataSharing data type.
5. Documentation
   : 1. Doxygen typos.
6. Repository
   : 1. Remove 2.9.x as active branch.
7. Non Tier 1 support
   : 1. Fixed SHM in 32-bit architectures.
     2. Fix warning on Win32 architecture.

#### NOTE
Upgrading to version 2.12.0 **requires** to regenerate generated source from IDL files using
[Fast DDS-Gen v3.0.1](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.0.1).

<a id="version-2-11-eol"></a>

### Version 2.11 (EOL)

<a id="id47"></a>

#### [Version 2.11.3 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.11.3/index.html)

This release includes the following **features** in an ABI compatible manner:

> 1. Support `Autofill port` ([automatically set a port](../../fastdds/transport/tcp/tcp.md#transport-tcp-transportdescriptor)) for TCP Transport.
> 2. Define a [super client](../../fastdds/env_vars/env_vars.md#env-vars-ros-super-client) by environment variable
> 3. Support [TCP Discovery server](../../fastdds/use_cases/tcp/tcp_with_discovery_server.md#use-case-tcp-discovery-server) CLI and environment variable
> 4. Define methods ([environment variable](../../fastdds/env_vars/env_vars.md#env-vars-builtin-transports),
>    [rtps layer](../../fastdds/rtps_layer/rtps_layer.md#rtps-layer-builtin-transports), [xml](../../fastdds/xml_configuration/domainparticipant.md#rtps)) to
>    [configure transport scenarios](../../fastdds/transport/tcp/tcp.md#transport-tcp-enabling)
> 5. Custom pools on DDS layer ([DataWriter](../../fastdds/dds_layer/publisher/dataWriter/createDataWriter.md#dds-layer-publisher-datawriter-with-payload-pool-creation) and
>    [DataReader](../../fastdds/dds_layer/subscriber/dataReader/createDataReader.md#dds-layer-subscriber-datareader-with-payload-pool-creation))

This release includes the following **improvements**:

> 1. Allow participant profiles with no rtps tag
> 2. Add Log warning message upon receiver resource creation failure, instead of an error
> 3. Updatable disable_positive_acks period
> 4. Backport workflows from master
> 5. Update GitUtils.cmake
> 6. Use foonathan memory manager for reducing allocations in `SharedMemManager.hpp`
> 7. Rerun failed tests with ctest option instead of colcon’s
> 8. Add CCache to all CI jobs
> 9. Simplify code in `CDRMessage`
> 10. TCP unique client announced local port
> 11. Make DataWriters always send the key hash on keyed topics
> 12. Include terminate process signals handler in discovery server

This release includes the following **fixes**:

> 1. Fix encapsulation format in WLP used for the `ParticipantMessageData`
> 2. Fix `DomainParticipant::register_remote_type` return when negotiating type
> 3. Fix `RemoteBuiltinEndpointHonoring` blackbox test
> 4. Fix .repos branches
> 5. Fix bad-free when receiving malformed DATA submessages
> 6. Fix clang warnings
> 7. Use STL implementation of `Timed/RecursiveTimedMutex` when `MSVC >= 19.36`
> 8. Fix the clang build for clang 14
> 9. Fix `HelloWorld` `DataSharing` example idl
> 10. Use `FASTRTPS_NO_LIB` on unittest root folder
> 11. Fix `Data Race` when updating liveliness changed in WLP
> 12. Fix TCP sender resources creation
> 13. Fix flow controllers unit tests compilation when using `Fast CDR` from thirdparty
> 14. Add XML parser `bit_bound` bounds check
> 15. Fix branch selection on Github CI
> 16. Better handling of trigger events in docs CI
> 17. Use `SO_EXCLUSIVEADDRUSE` for Win32 unicast listening sockets
> 18. Fix `PubSubAsReliable` test
> 19. Fix `FileWatchTest` for Github windows CI
> 20. Fix mac address overflow on windows
> 21. Fix missing mandatory attribute check in XML parser struct type
> 22. Update TLS unit test certificates
> 23. Add missing thread include
> 24. Add tests for reconnection with same `GUID`
> 25. Notify data-sharing listener at the end of a successful matching in intraprocess
> 26. Fix TCP deadlock on channel reuse
> 27. TCP non-blocking send
> 28. Fix DNS filter in CMakeLists file for tests
> 29. Fix bad-free when receiving malformed DATA_FRAG submessage
> 30. Fix memory problem when ciphering payload
> 31. Fix CVE-2023-50257
> 32. Fix build with TLS, but not security
> 33. Fix std::move warning
> 34. Update PR template to include check for PR description, title and backports
> 35. Fix data race on writer destruction while sending heartbeat
> 36. Fix comparison in `remove_from_pdp_reader_history`
> 37. Fix data race in PDPListener and SecurityManager
> 38. Fix an uninitialized value when building with GCC 13.2.0
> 39. Fix max clash with Windows CI
> 40. Discard already processed samples on PDPListener
> 41. Remove unnecessary TCP warning
> 42. Fix wrong log info messages on TCP
> 43. Revert “TCP deadlock on channel reuse”
> 44. Return const reference in `get_log_resources`
> 45. Add a keyed fragmented change to the reader data instance only when it is completed
> 46. Fix and refactor Windows Github CI

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id48"></a>

#### [Version 2.11.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.11.2/index.html)

This release includes the following **improvements**:

1. Improve Shared Memory resilience to crashing participants
2. User configuration for [Shared Memory metatraffic](../../fastdds/property_policies/non_consolidated_qos.md#property-policies-shm-enforce-metatraffic)
3. Performance improvements on intraprocess and data-sharing

This release includes the following **fixes**:

1. Remove Mutex from TimedEventImpl
2. Replace uint64_t by 8 in `alignas` specifier
3. Fix XMLParser null-dereference in `parseLogConfig`

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id49"></a>

#### [Version 2.11.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.11.1/index.html)

This release includes the following **improvements**:

> 1. Correct CONTRIBUTING.md typo
> 2. Improve validation on PID_PROPERTY_LIST deserialization
> 3. Apply eProsima brand style to Fast DDS repository
> 4. Fix spelling mistake: SUBSTRACTION to SUBTRACTION

This release includes the following **fixes**:

> 1. Fixed long-standing reconnection issue on SHM transport
> 2. Added missing include
> 3. Fixed Boost handle usage regression
> 4. Fix StatelessWriter locators filtering
> 5. Avoid double definition of FASTDDS_ENFORCE_LOG_INFO
> 6. Explicitly register type object in ContentFilteredTopicExample
> 7. Properly handle zero-sized payloads on dynamic memory payload pools

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id50"></a>

#### [Version 2.11.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.11.0/index.html)

This release includes the following **features**:

1. [Ignore every local endpoint within the DomainParticipant preventing local matching](../../fastdds/property_policies/ignore_local_endpoints.md#property-ignore-local-endpoints).
2. Extend DynamicDataHelper API providing a `print` overload with `std::ostream` as parameter.
3. TypeLookup Service settings.
4. Static Discovery XSD Schema.

This release includes the following **improvements**:

1. **Fast DDS improvements**
   : 1. Assign minimum available participant ID to new participants.
     2. Export symbols correctly on ContentFilteredTopic.
     3. Improve content filter expression parameters check and verbosity.
     4. Check TCP headers endianness.
     5. Security module: distinguished names (DN) comparison.
2. **Fast DDS deprecation**
   : 1. DDS:Crypto:AES-GCM-GMAC configuration using Property Policy QoS (security vulnerability).
3. **CI improvements**
   : 1. Include BitmapRange unit tests.
     2. Support for running some tests in parallel.
     3. Windows workflow.
4. **Build system**
   : 1. Improve CMake target loading. Removal of `FASTDDS_STATIC` CMake option.
     2. Avoid auto-linkage using CMake.
5. **Dependencies**
   : 1. Upgrade internal type supports using latest Fast DDS-Gen release v2.5.1.
     2. Upgrade Fast CDR submodule to v1.1.0.
6. **Examples**
   : 1. Admit XML configuration files in AdvanceConfigurationExample.
     2. New Discovery Server example.

This release includes the following **fixes**:

1. **Fast DDS bugfixes**
   : 1. Fix crash when creating two participants with the same fixed participant ID.
     2. Fix crash when calling
        [`on_requested_deadline_missed()`](../../fastdds/api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener28on_requested_deadline_missedEP10DataReaderRK29RequestedDeadlineMissedStatus)
        callback.
     3. Fix crashes caused by not capturing every Fast CDR exception.
     4. Correctly resolve aliases in DDSSQLFilter.
     5. Wait for log background thread initialization on the first queued entry.
     6. Fix data race when accessing `WRITE_PARAM_DEFAULT` static variable.
     7. Fix partition copy in QoS.
     8. Fix Data-Sharing delivery when data_count is zero.
     9. Fix API Fast DDS v2.10.0 API break calling correctly.
        [`on_participant_discovery()`](../../fastdds/api_reference/dds_pim/domain/domainparticipantlistener.md#_CPPv4N8eprosima7fastdds3dds25DomainParticipantListener24on_participant_discoveryEP17DomainParticipantN7fastdds4rtps26ParticipantDiscoveryStatusERK27ParticipantBuiltinTopicDataRb)
        callbacks.
     10. Security module: Honor [Allow Unauthenticated Participants](../../fastdds/security/access_control_plugin/access_control_plugin.md#allow-unauthenticated-section) flag.
     11. Fix concurrent access to
         [`load_profiles()`](../../fastdds/api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory13load_profilesEv).
     12. Fix UBSan (Undefined Behavior Sanitizer) issues.
     13. Improve Doxygen documentation about DomainParticipantListener discovery callbacks.
2. XSD fixes
   : 1. Set TransportDescriptor kind parameter as optional.
     2. Correctly assign QoS to the proper endpoint.
     3. Add missing tags.
3. CI fixes
   : 1. Fix null dereference in fuzzer code.
     2. Limit Thread Sanitizer memory usage to prevent runner shutdown.
     3. Use correct time unit in latency tests.
     4. Run communication tests.
4. Examples
   : 1. Correct DDS entity deletion order.
5. Installer generation
   : 1. Add documentation fallback when the documentation tag is not found.
6. Repository
   : 1. Remove 2.1.x as active branch.
     2. Remove 2.8.x as active branch.
7. Non Tier 1 support
   : 1. Fix build on MSVC 19.36.
     2. Forward compatibility with Boost inter-process 1.74+.
     3. Include missing header files required for compiling with GCC 13.
     4. QNX build fixes.
     5. Fix build issues in RPM systems.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="version-2-10-eol"></a>

### Version 2.10 (EOL)

<a id="id51"></a>

#### [Version 2.10.7 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.10.7/index.html)

This release includes the following **fixes**:

1. Fix [CVE-2025-24807](https://www.cve.org/CVERecord?id=CVE-2025-24807)
2. Release `participant_stateless` secure builtin writer history change when authentication has finished
3. Improve `OpenSSL` lifecycle handling
4. Fix comparison in `is_update_allowed`
5. Arithmetic overflow in fragment size calculations
6. Fix `-Werror=template-id-cdtor`
7. Fix double-locking issue in `DataSharingListener`
8. Improve `PDPClient` initialization
9. Fix unique network flows with TCP transports
10. Decouple transport receivers creation using unique network flows
11. Fix EDP reliability timings
12. Fix `several_writers_on_unack_sample_removed` flaky test
13. Reliable volatile change dropped when all history acked
14. Create initial connection for TCP initial peers
15. Filter interested readers on PDP writer
16. Fix cleanup race condition with exclusive and shared lock files
17. Fix TCP discovery server locators translation
18. Use correct algorithm strings on `PermissionsToken` and `IdentityToken`
19. Fix potential deadlock between `StatefulWriter` and `FlowController`
20. Fix log category name macro collision in `MacOS`
21. Unacknowledged sample removed in KeepAll mode
22. Fix assertion on `OutputTrafficManager`

This release includes the following **improvements**:

1. Log any errors before `cancel_init`
2. Improve `max_allocations` calculation on SHM transport
3. New property to select preferred key agreement algorithm
4. Update reception timestamp when it is added to the instance
5. Improve Blackbox TCP tests suite
6. Handle socket buffer size setting when system’s maximum exceeded
7. Address some compilation warnings with GCC latest
8. Refactor builtin writers & readers creation
9. Update sqlite from 3.36.0 to 3.47.2
10. Regenerate code with Fast DDS Gen v2.5.3

Github CI management:

1. Force Asio thirdparty in MacOS CI
2. Get correct Fast CDR related branch in CI
3. Update submodules when cloning Fast DDS on CI

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id52"></a>

#### [Version 2.10.6 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.10.6/index.html)

This release includes the following **fixes**:

1. Remove double `//` in some installation paths
2. Discard changes with big key-only payload and no key hash
3. Fix DataReaderHistory regression
4. Be less strict with parameter lengths
5. Check if `SHM` transport is disabled in `LARGE_DATA` modes
6. Fix issues in Dynamic Network Interfaces

This release includes the following **improvements**:

1. Add references to new nightly jobs per supported branch in README
2. Update commercial support section in README
3. Modify help command of XML CLI tool

Github CI management:

1. Allow running CI on external contributions

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id53"></a>

#### [Version 2.10.5 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.10.5/index.html)

This release includes the following **features** in an ABI compatible manner:

1. Add XML configuration for [FlowControllerDescriptor](../../fastdds/xml_configuration/common.md#flowcontrollers-xml)

This release includes the following **improvements**:

1. Use `%*` instead of loop in `.bat` scripts
2. Documentation improvements
3. Use absolute paths when loading XML files
4. Reduce tests flakiness
5. Add unsigned specification to literals

Github CI management:

1. Set fallback branch for `get_related_branch_from_repo` correctly
2. Fix sanitizers CI test summary report
3. Run selected VS tool on Windows CI
4. Use token for CCache action
5. Refactor Fast DDS Ubuntu CI to include several tests
6. Avoid CCache in some Github workflows
7. Build profiling tests as alternate build in Ubuntu CI
8. Add Ubuntu weekly CI
9. Fix python version in sanitizers CI
10. Fix windows CI and add vanilla build step in Ubuntu CI
11. Use eProsima-CI action to install Qt
12. Update types regeneration script homing path

This release includes the following **fixes**:

1. Automatically unmatch remote participants on participant deletion
2. Only apply content filter to ALIVE changes
3. Fix liveliness state in a multiple reader - one writer scenario
4. Fix topic interference on `liveliness_changed` status
5. Fix DS servers not connecting due to ports logic
6. Fix issue with exclusive ownership and unordered samples
7. Fix SecurityManager memory issue
8. Correctly initialize `MatchingFailureMask` constants to be used with the `std::bitset` API
9. Fix data race in `TypeObjectFactory::get_instance`
10. Properly delete builtin statistics writers upon `delete_contained_entities()`
11. Fix secure simple participants with initial peers not matching over TCP
12. Fix access violations on XML parser detected by OSS-fuzz

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id54"></a>

#### [Version 2.10.4 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.10.4/index.html)

This release includes the following **features** in an ABI compatible manner:

1. TCP Client and Server Participant Decision Making.
2. [Authentication Handshake Properties](../../fastdds/property_policies/security.md#property-policies-security) documentation.
3. New [max_message_size property](../../fastdds/property_policies/non_consolidated_qos.md#property-max-message-size) to limit output datagrams size.

This release includes the following **improvements**:

1. Return const reference in `get_log_resources`.
2. Include variety of terminate process signals handler in discovery server.
3. Check History QoS inconsistencies.
4. Make DataWriters always send the key hash on keyed topics.
5. `LARGE_DATA` Participants logic with same listening ports.
6. Effectively assert `AUTOMATIC/MANUAL_BY_PARTICIPANT` liveliness.
7. Improve checklist on PR template.
8. Allow processing of `AckNack` submessages with `count == 0`.
9. Internal refactor on port handling.
10. Upgrade Fast CDR submodule to v1.0.28

TCP transport improvements:

1. TCP `non-blocking` send.
2. Enabling multiple interfaces through whitelist in TCP servers.
3. Set real TCP `non-blocking-send` limitation.
4. Clean up TCP send resources on peer disconnection.

Github CI management:

1. Add macOS Github CI.
2. Avoid running GitHub CI if PR has conflicts.
3. Add Ubuntu Github CI.
4. Improve CI version management.
5. Pin CMake version and `vm.mmap_rnd_bits` in sanitizer workflows.
6. Only run PRs CI when review is requested.
7. Refactor Github CI sanitizer related jobs.
8. Build `ShapesDemo` on Ubuntu Github CI.
9. Fix Python version and environment.
10. Add DNS entries to hosts files on Github workflows.
11. Build Fast DDS Python bindings in Fast DDS Docs Github CI job.

This release includes the following **fixes**:

1. Fix and refactor Windows Github CI.
2. Fix wrong log info messages on TCP.
3. Add a keyed fragmented change to the reader data instance only when complete.
4. Prevent index overflow and correctly assert the end iterator in DataSharing.
5. Fix the shared memory cleaning script.
6. Fix CI documentation workflow label triggering.
7. Add missing virtual destructor for `StatisticsAncillary`.
8. Migrate apt package installation action to `eProsima-CI`.
9. Add missing `TypeLookup` listeners.
10. Fix doxygen docs warnings. Prepare for compiling with `Doxygen 1.10.0`.
11. Upgrade dependency version to last patch version in `.repos` file.
12. Fix TCP reconnection after open logical port failure.
13. Avoid unhandled asio exceptions.
14. Fix `CVE-2024-28231`.
15. Fix data race on PDP.
16. Fix flaky Log tests.
17. Fix some flaky MacOS tests.
18. Fix hidden overloaded virtual methods.
19. Fix test filtering in CMake files.
20. Avoid first message loss in TCP.
21. Fix `CVE-2024-30258 / CVE-2024-30259`.
22. Enforce SHM ports open mode exclusions.
23. Force unlimited [ResourceLimits](../../fastdds/dds_layer/core/policy/standardQosPolicies.md#resourcelimitsqospolicy) if lower or equal to zero.
24. Removed warning in `ParameterList`.
25. Make [`DataReader::get_first_untaken_info()`](../../fastdds/api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader22get_first_untaken_infoEP10SampleInfo) coherent with `read()/take()`.
26. Fix leak in `SecurityManager`.
27. Fix support for `@key` annotation in `DynamicTypes`.
28. Fix leaks in XML parser for `DynamicTypes`.
29. Fix Discovery Server over TCP.
30. Handle errors when setting socket buffer sizes.
31. Fix [`on_sample_lost`](../../fastdds/api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener14on_sample_lostEP10DataReaderRK16SampleLostStatus) notification on best-effort readers for fragmented samples.
32. Fix DataSharing QoS deserialization.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id55"></a>

#### [Version 2.10.3 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.10.3/index.html)

This release includes the following **features** in an ABI compatible manner:

1. Support `Autofill port` ([automatically set a port](../../fastdds/transport/tcp/tcp.md#transport-tcp-transportdescriptor)) for TCP Transport.
2. Define a [super client](../../fastdds/env_vars/env_vars.md#env-vars-ros-super-client) by environment variable
3. Support [TCP Discovery server](../../fastdds/use_cases/tcp/tcp_with_discovery_server.md#use-case-tcp-discovery-server) CLI and environment variable
4. Define methods ([environment variable](../../fastdds/env_vars/env_vars.md#env-vars-builtin-transports),
   [rtps layer](../../fastdds/rtps_layer/rtps_layer.md#rtps-layer-builtin-transports), [xml](../../fastdds/xml_configuration/domainparticipant.md#rtps)) to
   [configure transport scenarios](../../fastdds/transport/tcp/tcp.md#transport-tcp-enabling)
5. Custom pools on DDS layer ([DataWriter](../../fastdds/dds_layer/publisher/dataWriter/createDataWriter.md#dds-layer-publisher-datawriter-with-payload-pool-creation) and
   [DataReader](../../fastdds/dds_layer/subscriber/dataReader/createDataReader.md#dds-layer-subscriber-datareader-with-payload-pool-creation))

This release includes the following **improvements**:

1. Log warning upon receiver resource creation failure
2. Simplify code in `CDRMessage`
3. Backport workflows from master
4. Rerun failed tests with ctest option instead of colcon’s
5. Use foonathan memory manager for reducing allocations in `SharedMemManager.hpp`
6. Add CCache to all CI jobs

This release includes the following **bugfixes**:

1. Fix `RemoteBuiltinEndpointHonoring` blackbox test
2. Fix bad-free when receiving malformed DATA submessage
3. Fix clang warnings
4. Use STL implementation of `Timed/RecursiveTimedMutex` when `MSVC >= 19.36`
5. Notify data-sharing listener at the end of a successful matching in intraprocess
6. Fix the clang build for clang 14
7. Fix `HelloWorld` Data-Sharing example idl
8. Fix the behaviour of `disable_positive_acks` period
9. Fix `DomainParticipant::register_remote_type` return when negotiating type
10. Fix Data Race when updating liveliness changed in WLP
11. Fix TCP sender resources creation
12. Fix flow controllers unit tests compilation when using `Fast CDR` from thirdparty
13. Add XML parser `bit_bound` bounds check
14. Add tests for reconnection with same GUID
15. Fix Github Windows CI
16. Fix PubSubAsReliable test
17. Use `FASTRTPS_NO_LIB` on unittest root folder
18. Fix missing mandatory attribute check in XML parser struct type
19. Fix mac address overflow on windows
20. Use `SO_EXCLUSIVEADDRUSE` for Win32 unicast listening sockets
21. Fix FileWatchTest for Github windows CI
22. Add missing thread include
23. Update TLS unit test certificates
24. Select correct .repos file on push events
25. Fix documentation CI branch
26. Fix TCP deadlock on channel reuse
27. Fix DNS filter in CMakeLists file for tests
28. Fix bad-free when receiving malformed DATA_FRAG submessage
29. Fix memory problem when ciphering payload
30. Fix build with TLS, but not security
31. Fix CVE-2023-50257
32. Fix data race on writer destruction while sending heartbeat
33. Fix comparison in `remove_from_pdp_reader_history`
34. Fix data race in PDPListener and SecurityManager
35. Update PR template to include check for PR description, title and backports
36. Fix std::move warning
37. Revert “TCP deadlock on channel reuse”
38. Fix max clash with Windows CI
39. Remove unnecessary TCP warning
40. Discard already processed samples on `PDPListener`
41. TCP unique client announced local port

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id56"></a>

#### [Version 2.10.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.10.2/index.html)

This release includes the following **improvements**:

1. Fix Data-Sharing delivery when data_count is zero
2. Improve performance of intraprocess plus data-sharing
3. Improve content filter expression parameters checks and verbosity
4. Improve validation on PID_PROPERTY_LIST deserialization
5. Participant ignore local endpoints
6. Pick smallest available participant ID for new participants
7. Improve endpoint QoS XML tags
8. Forward compatibility with boost interprocess 1.74+
9. Cap Thread Sanitizer memory usage to prevent runner shutdown
10. Allow participant XML profile with no <rtps> tag
11. Add unsupported note in API documentation to new ignore DomainParticipantListener callbacks
12. Add documentation version fallback

This release includes the following **bugfixes**:

1. Fixed long-standing reconnection issue on SHM transport
2. Fix null dereference when fuzzing
3. Fix segfault when creating two participant with same fixed id
4. Fix UBSan (Undefined Behavior Sanitizer) issues
5. Fix listener selection for on_requested_deadline_missed
6. Fix build on msvc 19.36.32528
7. Fix XML schema to set Transport descriptor kind as NOT mandatory
8. Fix missing includes
9. Fix overhead time unit
10. Fix request reply example spelling typo
11. Fix topic deletion after endpoint in examples
12. Fix Data-Sharing delivery when data_count is zero
13. Wait for log background thread initialization on the first queued entry
14. Fix alias resolve in DDSSQLFilter
15. Fix partition copy in QoS
16. Fix StatelessWriter locators filtering
17. Fix XMLParser null-dereference in parseLogConfig
18. Fix encapsulation format in WLP
19. Replace uint64_t by 8 in alignas specifier
20. Capture all Fast CDR exceptions
21. Security module: Honor allow_unauthenticated_participants flag
22. Explicitly register type object in ContentFilteredTopicExample
23. Avoid double definition of FASTDDS_ENFORCE_LOG_INFO
24. Fix API Fast DDS v2.10.0 API break calling correctly on_participant_discovery callbacks
25. Remove mutex from TimedEventImpl

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id57"></a>

#### [Version 2.10.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.10.1/index.html)

This release includes the following **feature** in an ABI compatible way:

1. [`ignore_participant()`](../../fastdds/api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant18ignore_participantERK16InstanceHandle_t) implementation

This release includes the following **bugfixes**:

1. Fix repeated matched event notification.
2. Fix regression introduced by #3396.
3. Initial acknack backoff.
4. Fix regular expression in XML schema.
5. Correctly assign multicast port to multicast initial peers.

This release includes the following **CI improvements**:

1. Update runner and GCC version for Thread Sanitizer job.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id58"></a>

#### [Version 2.10.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.10.0/index.html)

This minor release includes several new features, improvements and bugfixes.

#### NOTE
Mind that, even though this release is API compatible with previous v2.x versions, it is *NOT* ABI compatible with
previous versions.
This means that applications upgrading Fast DDS to v2.10.0 will require recompilation, though not source code
modification.

#### NOTE
It is also advisable to regenerate the type support from the IDL files using
[Fast DDS-Gen v2.4.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v2.4.0).
Furthermore, if upgrading to v2.10.0, it is also recommended to upgrade Fast CDR to
[v1.0.27](https://github.com/eProsima/Fast-CDR/releases/tag/v1.0.27).

This release includes the following **features**:

1. New [on_unacknowledged_sample_removed callback](../../fastdds/dds_layer/publisher/dataWriterListener/dataWriterListener.md#dds-layer-publisher-datawriterlistener-on-unack-sample-removed) in
   [`DataWriterListener`](../../fastdds/api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListenerE).
2. [Secure Discovery Server](../../fastdds/discovery/discovery_server.md#ds-security).
3. DomainParticipant ignore empty API.
4. RTPS `ReaderListener::on_incompatible_type`
   and `WriterListener::on_incompatible_type`
   empty API.

This release includes the following **improvements**:

1. **Fast DDS improvements**
   : 1. Improve behavior when `STRICT_REALTIME` [CMake option](../../installation/configuration/cmake_options.md#cmake-options) is not enabled.
     2. Using functors for `for_matched_readers` parameter.
     3. Improve auto GAPs in Data Sharing.
     4. Use standard value for `PID_RELATED_SAMPLE_IDENTITY`.
2. **Contributions and repository quality**
   : 1. Update Pull Request template.
     2. Update foonathan_memory quality declaration.
     3. Update XSD schema.
     4. Make network headers private avoiding exposing non-public API.
     5. Improve Doxygen documentation for
        [`ResourceLimitsQosPolicy`](../../fastdds/api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicyE).
3. **Examples**
   : 1. New [Request-Reply example](../../fastdds/use_cases/request_reply/request_reply.md#use-case-request-reply).
4. **CI improvements**
   : 1. New workflow to check documentation build.
     2. ASAN workflow updated to use Ubuntu 22.04.
5. **Dependencies**
   : 1. Upgrade internal type supports using latest Fast DDS-Gen release v2.4.0.
     2. Upgrade Fast CDR submodule to v1.0.27.
6. **Fast DDS CLI**
   : 1. Handle `SIGTERM` signal.
7. **Community supported platforms**
   : 1. QNX 7.1.

This release includes the following **fixes**:

1. **Security vulnerability**
   : 1. Fix chain of trust issues with a single CA certificate.
2. **Bugfixes**
   : 1. Fix RTPS StatelessWriter ACK check.
     2. ASAN (Address Sanitizer) fixes.
     3. UBSan (Undefined Behavior Sanitizer) fixes.
     4. Export public API correctly in Windows.
     5. Correctly handle builtin endpoints mask.
     6. Fix backwards compatibility using SHM communication.
     7. Protect against uncaught exception in SHM segment creation.
     8. Fix build for GCC 5.
     9. Validity check for first sequence number.
     10. Fix crash when enabling DisablePositiveACKsQoSPolicy with remote best-effort readers.
3. **Synchronization fixes**
   : 1. Take mutex when removing local reader in WLP.
     2. Fix data races in SecurityManager authentication process.
4. **CI fixes**
   : 1. Fix test building when using `GTEST_INDIVIDUAL` [CMake option](../../installation/configuration/cmake_options.md#cmake-options).
     2. Fix overflow in received samples in performance tests.
5. **Example fixes**
   : 1. Avoid creating entities within callbacks in DynamicHelloWorldExample.
     2. Remove Asio dependency from DeadlineQoSExample.
6. **Repository fixes**
   : 1. Remove 2.7.x as active branch.
7. **Community supported platforms**
   : 1. Include right header when building for iOS.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="version-2-9-eol"></a>

### Version 2.9 (EOL)

<a id="id59"></a>

#### [Version 2.9.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.9.2/index.html)

This release includes the following ABI compatible **improvements**:

1. **Library improvements**
   : 1. Improve behavior when `STRICT_REALTIME` [CMake option](../../installation/configuration/cmake_options.md#cmake-options) is not enabled.
     2. Using functors for `for_matched_readers` parameter.
     3. Improve auto GAPs in Data Sharing.
     4. Improve content filter expression parameters check and verbosity.
     5. Improve validation on `PID_PROPERTY_LIST` deserialization.
2. **Fast DDS CLI**
   : 1. Handle `SIGTERM` signal.

This release includes the following **bugfixes**:

1. Security vulnerability
   : 1. Fix chain of trust issues with a single CA certificate.
2. Library bugfixes
   : 1. Fix RTPS StatelessWriter ACK check.
     2. UBSan (Undefined Behavior Sanitizer) fixes.
     3. Fix backwards compatibility using SHM communication.
     4. Correctly handle builtin endpoint mask.
     5. Fix crash when enabling DisablePositiveACKsQosPolicy with remote best-effort readers.
     6. Validity check for first sequence number.
     7. ASAN (Address Sanitizer) fixes.
     8. Correctly assign multicast port to multicast initial peers.
     9. Protect against uncaught exception in SHM segment creation.
     10. Initial acknack backoff.
     11. Fix crash when calling
         [`on_requested_deadline_missed()`](../../fastdds/api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener28on_requested_deadline_missedEP10DataReaderRK29RequestedDeadlineMissedStatus)
         callback.
     12. Security module: Honor `allow_unauthenticated_participants` flag.
     13. Fix crashes caused by not capturing every Fast CDR exception.
     14. Correctly resolve aliases in DDSSQLFilter.
     15. Wait for log background thread initialization on the first queued entry.
     16. Fix partition copy in QoS.
     17. Fix Data-Sharing delivery when `data_count` is zero.
     18. Fix StatelessWriter locators filtering.
     19. Avoid double definition of `FASTDDS_ENFORCE_LOG_INFO`.
     20. Fixed long-standing reconnection issue on SHM transport.
3. CI fixes
   : 1. Fix test building when using `GTEST_INDIVIDUAL` [CMake](../../installation/configuration/cmake_options.md#cmake-options) option.
     2. Use correct time unit in latency tests.
4. Synchronization fixes
   : 1. Take mutex when removing local reader in WLP.
     2. Fix data races in SecurityManager authentication process.
5. Example fixes
   : 1. Avoid creating entities within callbacks in DynamicHelloWorldExample.
     2. Remove Asio dependency from DeadlineQosExample.
     3. Correct DDS entity deletion order.
     4. Explicitly register TypeObject in ContentFilteredTopicExample.
6. Installer generation
   : 1. Add documentation fallback when the documentation tag is not found.
7. Non Tier 1 Support
   : 1. Fix build for GCC5.
     2. Fix build on MSVC 19.36.
     3. Include right header when building for iOS.
     4. Forward compatibility with Boost inter-process 1.74+.
     5. Include missing header files required for compiling with GCC13.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id60"></a>

#### [Version 2.9.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.9.1/index.html)

This release includes the following **bugfixes**:

1. **Synchronization fixes**:
   : 1. Fix deadlock in Writer Liveliness Protocol (WLP) using intraprocess.
     2. Fix data race in
        [`DomainParticipant::set_listener()`](../../fastdds/api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant12set_listenerEP25DomainParticipantListener).
     3. Fix deadlock on TLS closure.
2. **Other fixes**:
   : 1. Fix notification lost.
     2. Fix `total_read_` to be consistent with Reader’s History after
        [`DataReader::get_first_untaken_info()`](../../fastdds/api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader22get_first_untaken_infoEP10SampleInfo).
     3. Use shared pointers for internal singletons.
     4. Support CCache on Windows.
     5. Avoid null dereference on fuzzer.
     6. Other minor fixes and improvements.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id61"></a>

#### [Version 2.9.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.9.0/index.html)

This minor release includes several new features (new log macros to avoid conflicts with external libraries), behavior
change (default memory management policy is no longer
`PREALLOCATED_MEMORY_MODE`,
enable by default Fast DDS Statistics module but only taking statistics metrics if the corresponding Statistics
DataWriter is enabled), performance improvements, CI improvements (including address sanitizer jobs), and several bug
fixes.

#### NOTE
Mind that, even though this release is API compatible with previous v2.x versions, it is *NOT* ABI compatible with
previous versions.
This means that applications upgrading Fast DDS to v2.9.0 will require recompilation, though not source code
modification.

#### NOTE
It is also advisable to regenerate the type support from the IDL files using
[Fast DDS-Gen v2.3.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v2.3.0).
Furthermore, if upgrading to v2.9.0, it is also recommended to upgrade Fast CDR to
[v1.0.26](https://github.com/eProsima/Fast-CDR/releases/tag/v1.0.26).

This release includes the following **features**:

1. New log macros [`EPROSIMA_LOG_INFO`](../../fastdds/api_reference/log/loginfo.md#c.EPROSIMA_LOG_INFO),
   [`EPROSIMA_LOG_WARNING`](../../fastdds/api_reference/log/logwarning.md#c.EPROSIMA_LOG_WARNING) and [`EPROSIMA_LOG_ERROR`](../../fastdds/api_reference/log/logerror.md#c.EPROSIMA_LOG_ERROR).
2. Add `ENABLE_OLD_LOG_MACROS` CMake option to support
   [disabling the compilation of previous log macros](../../fastdds/logging/disable_module.md#old-log-macros-disable).

This release includes the following **behavior changes**:

1. Default [memory management policy](../../fastdds/dds_layer/core/policy/eprosimaExtensions.md#memorymanagementpolicy) set to
   `PREALLOCATED_WITH_REALLOC_MEMORY_MODE`.
2. Statistics metrics are only calculated/accumulated when their corresponding DataWriter is enabled in
   [Fast DDS Statistics Module](../../fastdds/statistics/dds_layer/domainParticipant.md#statistics-enable-datawriters).
3. Enable `FASTDDS_STATISTICS` [CMake option](../../installation/configuration/cmake_options.md#cmake-options) by default.

This release includes the following **improvements**:

1. CI improvements:
   : 1. Add address sanitizer job for Fast DDS library.
     2. Add address sanitizer job for Discovery Server test suite.
2. Upgrade Fast CDR submodule.

This release includes the following **bugfixes**:

1. Synchronization fixes:
   : 1. Fix deadlock when removing remote DomainParticipants by expired liveliness when using Security.
     2. Protect [`DomainParticipant::set_listener()`](../../fastdds/api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant12set_listenerEP25DomainParticipantListener)
        avoiding null reference.
     3. Fix data race on `WriterProxy::stop` while TimedEvent is being triggered.
     4. Protect creation/destruction of Boost’s `named_mutex`.
2. CI fixes:
   : 1. Fix Statistics Module test suite.
     2. Fix recurring data races in test suite.
     3. Fix thread sanitizer job keeping Ubuntu 20.04.
3. Other:
   : 1. Fix Topic creation when registering a type name different from the internal TypeSupport name.
     2. Fix communication with asymmetric [Ignore Participant flags](../../fastdds/discovery/general_disc_settings.md#discovery-ignore-flags).
     3. Several dependencies fixes upgrading to Ubuntu 22.04.
     4. Disable error logged when DomainParticipant profile is not found.
     5. Fix CMake for Fast DDS use as submodule.
     6. Upgrade internal type supports generated with Fast DDS-Gen v2.3.0.
     7. Other minor fixes.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="version-2-8-eol"></a>

### Version 2.8 (EOL)

<a id="id62"></a>

#### [Version 2.8.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.8.2/index.html)

This release includes the following **improvements**:

1. Add ASAN CI tests for Fast DDS and Discovery Server
2. Mirror master onto both 2.8.x & 2.9.x
3. Doxygen documentation: add deprecation notice to ThroughputControllerDescriptor
4. Several fixes to remove warnings in Ubuntu Jammy (22.04)
5. Improve behavior when HAVE_STRICT_REALTIME is not set
6. Using functors in StatefulWriter.cpp for_matched_readers
7. Fix build on old compilers
8. Avoid creation of DynamicTypes on example
9. Implement a validity check for firstSN

This release includes the following **bugfixes**:

1. Fix bug in Topic creation with different Type Name
2. Fix tests failing with subprocess aborted error
3. Fix communication with asymmetric ignoreParticipantFlags
4. Added ignore_participant_flags() to Blackbox_FastRTPS PubSubReader.
5. Fix Deadlock in remove_participant (ResourceEvent thread) when compiled WITH_SECURITY
6. Fix failed tests when compiling with statistics enabled
7. Fix Windows StatistisQosTests.cpp linkage and Failed test
8. Fixing deadlock in WLP
9. Fix notification lost
10. Fix StatelessWriter ACK check
11. Fix total_unread_ consistent with reader’s history upon get_first_untaken_info()
12. Fix chain of trust issues
13. Fixed StatisticsSubmessageData unaligned access
14. Fix build error when GTEST_INDIVIDUAL is OFF
15. Correctly handle builtin endpoints mask
16. Added missing mutex to WLP::remove_local_reader
17. Handle SIGTERM in fast discovery server
18. Improve auto gaps in data sharing
19. Replaced SecurityManager temporary ProxyDatas with ProxyPools
20. Fix crash when disable_positive_acks is enable and the remote reader is best-effort
21. Protect from uncaught exception during SHM Segment creation
22. Fix asio dependency
23. Include the right header when building for iOS

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id63"></a>

#### [Version 2.8.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.8.1/index.html)

This release includes the following **bugfixes**:

1. Statistics module fixes:
   : 1. Increase statistics DataWriter history.
     2. Fix Statistics module CI.
     3. Install Statistics IDL file.
     4. Fix for building in old compilers.
     5. Fix core dumped in
        [`delete_contained_entities()`](../../fastdds/api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant25delete_contained_entitiesEv).

1. Address sanitizer fixes:
   : 1. Add ASAN CI job and `SANITIZE` [CMake option](../../installation/configuration/cmake_options.md#cmake-options).
     2. Fixes reported by address sanitizer.

1. Synchronization fixes:
   : 1. Fix data races when creating DataWriters.
     2. Ensure `shared_mutex` implementation is consistent throughout supported platforms.

1. Other fixes:
   : 1. Include missing ReadCondition header.
     2. Fix selection of output locators.
     3. Fix null-dereference on `parseXMLEnumDynamicType`.
     4. Include `2.8.x` branch release support.
     5. Send GAPs correctly when using separate sending.
     6. Fixes for building in old compilers.
     7. Fix [`DataReader::read_next_instance()`](../../fastdds/api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader18read_next_instanceER18LoanableCollectionR13SampleInfoSeqNSt7int32_tERK16InstanceHandle_t15SampleStateMask13ViewStateMask17InstanceStateMask)
        and [`DataReader::take_next_instance()`](../../fastdds/api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader18take_next_instanceER18LoanableCollectionR13SampleInfoSeqNSt7int32_tERK16InstanceHandle_t15SampleStateMask13ViewStateMask17InstanceStateMask)
        implementation.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id64"></a>

#### [Version 2.8.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.8.0/index.html)

This minor release includes several new features, performance improvements (especially in the case of topics with many
instances), CI improvements (including the ability to run the CI in Android emulators or devices), and several bug
fixes.

#### NOTE
Mind that, even though this release is API compatible with previous v2.x versions, it is *NOT* ABI compatible with
previous versions.
This means that applications upgrading Fast DDS to v2.8.0 will require recompilation, though not source code
modification.

#### NOTE
It is also advisable to regenerate the type support from the IDL files using
[Fast DDS-Gen v2.2.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v2.2.0).
Furthermore, if upgrading to v2.8.0, it is also recommended to upgrade Fast CDR to
[v1.0.25](https://github.com/eProsima/Fast-CDR/releases/tag/v1.0.25).

This release includes the following **features**:

1. Full [Ownership](../../fastdds/dds_layer/core/policy/standardQosPolicies.md#ownershipqospolicy) and [OwnershipStrength](../../fastdds/dds_layer/core/policy/standardQosPolicies.md#ownershipstrengthqospolicy) QoS support
2. [External locators](../../fastdds/transport/announced_locators.md#external-locators)
3. UDPv6 support for [fast-discovery-server tool](../../fastddscli/cli/cli.md#cli-discovery) and
   [ROS_DISCOVERY_SERVER](../../fastdds/env_vars/env_vars.md#env-vars-ros-discovery-server)
4. [XML configuration support for statistics DataWriters QoS](../../fastdds/statistics/dds_layer/domainParticipant.md#statistics-enable-datawriters)
5. [SNI support](../../fastdds/transport/tcp/tls.md#transport-tcp-tls)
6. [Propagate PropertyQoS properties when explicitly set](../../fastdds/dds_layer/core/policy/eprosimaExtensions.md#propertypolicyqos)
7. Add API to createRTPSWriter with a custom pool
8. Add `std::string::compare` API to `fixed_string`
9. Get WAN address API in TCPv4 transport descriptors
10. Adding `DomainParticipantFactory::get_shared_instance()` API

This release includes the following **improvements**:

1. Performance improvements:
   : 1. Skip writer_removed processing for unaccounted instances
     2. Improve GUID_t operator< performance
2. CI improvements:
   : 1. Add optional parameters to thread-sanitizer job
     2. Enable Android testing on device
3. Examples:
   : 1. Update BasicConfigurationExample to allow set up TTL
     2. Add Guid info to BasicConfiguration Example `cout`
4. Internal implementation improvements:
   : 1. Add script to generate idl files
     2. Group set_qos_from_attributes free functions into a separate file
     3. Update script for generating idl files
     4. Set `last_heartbeat_count_` private member of WriterProxy as atomic
5. Android Improvements
6. Upgrade Fast CDR submodule

This release includes the following **bugfixes**:

1. Synchronization fixes:
   : 1. Fix datarace using writer’s locator selectors
     2. Add lock guard at changing SHM port listener status members
     3. Add atomic variable to prevent datarace in FlowController
     4. Disable RTPSParticipantImpl after removing it from RTPSDomain participants list
     5. Fixing datarace on listener callbacks
     6. Protect access to reader listeners
     7. Use thread-safe localtime function in unix distributions
     8. Fixed usage of uninitialised ifreq
     9. Adding protection to id_counter access
2. Repository fixes:
   : 1. Fix spelling mistake
     2. Add python3 dependency to package.xml
3. Other:
   : 1. Fix null dereference on parseXMLBitsetDynamicType
     2. Change internal include path of nlohmann/json header file
     3. Instance allocation consistency
     4. Fix complex member printing for DynamicDataHelper
     5. Fix initialization order in mock
     6. Upgraded internal type supports

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="version-2-7-eol"></a>

### Version 2.7 (EOL)

<a id="id65"></a>

#### [Version 2.7.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.7.2/index.html)

This release includes the following **improvements**:

1. Skip `writer_removed` processing for unaccounted instances.
2. Improve `GUID_t` operator `<` performance.

This release also includes the following **bugfixes**:

1. Fix complex member printing for DynamicDataHelper.
2. Add python3 dependency.
3. Fix selection of output locators.
4. Fix data races when creating DataWriters.
5. Fix null dereferences on XML parser.
6. Send GAPs correctly when using separate sending.
7. Install Statistics IDL file.
8. Fixes for building in old compilers.
9. Fix several deadlocks.
10. Fix communication with asymmetric [Ignore Participant flags](../../fastdds/discovery/general_disc_settings.md#discovery-ignore-flags).
11. Fix notification lost.
12. Fix StatelessWriter ACK check.
13. Fix `total_read_` to be consistent with Reader’s History after
    [`DataReader::get_first_untaken_info()`](../../fastdds/api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader22get_first_untaken_infoEP10SampleInfo).
14. Fix doxygen documentation adding deprecated to `ThroughputControllerDescriptor`.
15. Several dependencies fixes upgrading to Ubuntu 22.04.
16. Ensure `shared_mutex` implementation is consistent throughout supported platforms.
17. Fix StatisticsSubmessageData unaligned access.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id66"></a>

#### [Version 2.7.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.7.1/index.html)

This release includes the following **features** in an ABI compatible way:

1. [Checking STATIC EDP XML Files](../../fastdds/discovery/static.md#check-static-xml-file) by means of [`DomainParticipantFactory::check_xml_static_discovery()`](../../fastdds/api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory26check_xml_static_discoveryERNSt6stringE).
2. [ReadCondition](../../fastdds/dds_layer/core/waitsets/waitsets.md#dds-layer-core-readcondition) implementation.

This release includes the following **improvements**:

1. Thread sanitizer CI.
2. Overload [`get_unread_count()`](../../fastdds/api_reference/dds_pim/subscriber/datareader.md#_CPPv4NK8eprosima7fastdds3dds10DataReader16get_unread_countEv).
3. Improve read/take performance when using topic with a great number of keys.
4. Improve rediscovery on lossy environments.
5. New [CMake option](../../installation/configuration/cmake_options.md#cmake-options) USE_THIRDPARTY_SHARED_MUTEX.
6. Notify changes in bulk in RTPS readers.

This release includes the following **bugfixes**:

1. Fix Fast CDR submodule update to v1.0.24.
2. Fix access to some pointers.
3. Fixed validation on `ParameterPropertyList_t`.
4. Fixed acknowledgement in DataSharing.
5. Fixed wrong usage of `std::remove_if`.
6. Suppress OpenSSL 3.0 warnings.
7. Fixed race condition in Logging module.
8. Other minor fixes and improvements.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id67"></a>

#### [Version 2.7.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.7.0/index.html)

This release includes the following **improvements**:

1. Support for [DDS SampleRejectedStatus API](../../fastdds/dds_layer/core/status/status.md#dds-layer-core-status-samplerejectedstatus)
2. Support for DDS DataWriter methods:

> 1. [`DataWriter::write_w_timestamp()`](../../fastdds/api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter17write_w_timestampEPCKvRK16InstanceHandle_tRKN7fastdds3dds6Time_tE)
> 2. [`DataWriter::register_instance_w_timestamp()`](../../fastdds/api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter29register_instance_w_timestampEPCKvRKN7fastdds3dds6Time_tE)
> 3. [`DataWriter::unregister_instance_w_timestamp()`](../../fastdds/api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter31unregister_instance_w_timestampEPCKvRK16InstanceHandle_tRKN7fastdds3dds6Time_tE)
> 4. [`DataWriter::dispose_w_timestamp()`](../../fastdds/api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter19dispose_w_timestampEPCKvRK16InstanceHandle_tRKN7fastdds3dds6Time_tE)
1. Support for DDS [`find_topic()`](../../fastdds/api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant10find_topicERKNSt6stringERKN7fastdds3dds10Duration_tE)
2. Support for GCC 12
3. Upgrade CMake minimum requirement to 3.16.3
4. Add Windows DLL support to Dynamic Types API

Some **bugfixes** are also included:

1. Deadlocks and data races
2. Move deprecated OpenSSL cleanup function to match the right version

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="version-2-5-eol"></a>

### Version 2.5 (EOL)

<a id="id68"></a>

#### [Version 2.5.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.5.2/index.html)

This release includes the following **improvements**:

1. Support lowercase keywords and hexadecimal values on SQL filter.
2. Support for GCC 12.

This release includes the following **bugfixes**:

1. Fix MatchedStatus last_\*_handle.
2. Fix recommended statistics DataReaderQos to enable backwards compatibility.
3. Fix deadlocks and data races.
4. Fix empty partition validation checks.
5. Fix corner case with reliable writers and samples with a huge number of fragments.
6. Other minor fixes and improvements.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id69"></a>

#### [Version 2.5.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.5.1/index.html)

This release includes the following **improvements**:

1. [ContentFilterTopic filtering at the DataReader side](../../fastdds/dds_layer/topic/contentFilteredTopic/createContentFilteredTopic.md#dds-layer-topic-filtering-data-on-topic).
2. Release lifecycle.

This release includes the following **bugfixes**:

1. XML parser fixes.
2. Discovery Server fixes.
3. Fix DataSharing sample validation.
4. PKCS#11 support fixes.
5. Test fixes.
6. Doxygen documentation fixes.
7. GAP message fix.
8. Enable memory protection on DataSharing readers.
9. TCP reconnection issues.
10. Fix dynamic network interfaces feature.
11. Several Security module fixes.
12. STRICT_REALTIME fix.
13. Suppress OpenSSL 3.0 warnings.
14. Move optionparser to thirdparty.
15. Thread-safe access to endpoints collections.
16. MemberDescriptor fully qualified name.
17. Setting QoS fix.
18. Other minor fixes and improvements.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id70"></a>

#### [Version 2.5.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.5.0/index.html)

This minor release is API compatible with the previous minor release, but introduces **ABI breaks** on
two of the three public APIs:

* Methods and attributes have been added on several classes of the DDS-PIM high-level API, so indexes of
  symbols on dynamic libraries may have changed.
* Methods and attributes have been added on several classes of the RTPS low-level API, so indexes of
  symbols on dynamic libraries may have changed.
* Old Fast-RTPS high-level API remains ABI compatible.

This minor release includes the following **features**:

1. [Support for PKCS#11](../../fastdds/security/pkcs11_support/pkcs11_support.md#pkcs11-support) format URIs for private keys
2. Added interfaces for content filter APIs
3. Allow new [network interfaces to be detected at runtime](../../fastdds/use_cases/dynamic_network_interfaces/dynamic_network_interfaces.md#dynamic-network-interfaces)
4. New API on [DataWriter](../../fastdds/api_reference/dds_pim/publisher/datawriter.md#api-pim-datawriter) to wait for a specific instance to be acknowledged
5. Added interfaces for [concatenation of transports](../../fastdds/transport/transport_api.md#transport-transportapi-chaining)
6. Allow [XML profiles to be loaded from a string](../../fastdds/xml_configuration/making_xml_profiles.md#loadingapplyingprofiles)
7. Allow [disabling piggyback heartbeat](../../fastdds/xml_configuration/common.md#xml-disableheartbeatpiggyback) on XML and DataWriter QoS
8. New basic configuration example

It also includes the following **improvements**:

1. Working implementation of instance_state and view_state
2. Allow zero-valued keys
3. Made some type aliases public to ease python bindings integration
4. Improved performance by avoiding unnecessary payload copies for samples that are going to be rejected
5. Removed unnecessary headers from Log module public headers
6. Add support for Key annotation in TypeObjectFactory
7. Only export public symbols on non-windows platforms
8. Some documentation improvements

Some important **bugfixes** are also included:

1. Fixed payload pool handling on EDPSimple destructor
2. Fixed null dereference on XML parser
3. Correctly export XTypes related methods on Windows
4. Ensure correct boost singleton destruction order
5. Avoid warning when environment file filename is empty
6. Correctly set GUID of DataWriter and DataReader upon creation

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="version-2-4-eol"></a>

### Version 2.4 (EOL)

<a id="id71"></a>

#### [Version 2.4.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.4.2/index.html)

This release includes the following **improvements**:

1. Enable memory protection on [DataSharing](../../fastdds/transport/datasharing.md#datasharing-delivery) readers
2. Add const overload of `DataReader::guid()`
3. Set recommended statistics [DataReaderQos](../../fastdds/statistics/dds_layer/qos.md#statistics-datareader-qos) to `PREALLOCATED_WITH_REALLOC`
4. Allow fully qualified name on MemberDescriptor

This release includes the following bugfixes:

1. Fix and refactor EDPSimple destructor
2. Fix several build warnings on certain platforms
3. Fix OSS fuzz issues
4. Fix [TCP](../../fastdds/transport/tcp/tcp.md#transport-tcp-tcp) synchronization issues
5. Correct reporting of [MatchedStatus](../../fastdds/dds_layer/core/status/status.md#dds-layer-core-status) `last_*_handle`
6. Ensure correct boost singleton destruction order
7. Fix inserting minimum CacheChange_t in GAP message

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*.

<a id="id72"></a>

#### [Version 2.4.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.4.1/index.html)

This release includes the following **improvements**:

1. Fixed several flaky tests
2. Improved bandwidth usage of GAPs and HEARTBEATs
3. Correctly implement delete_contained_entities
4. Use native inter-process on Windows
5. Improved performance of unregister_instance
6. Improved OSS-fuzz integration
7. Support for partitions on DataWriterQoS and DataReaderQoS
8. Some documentation improvements
9. Removed unused macro to avoid naming clashes

This release includes the following **bugfixes**:

1. Avoid bad_node_size exception when cross building
2. Fixed build on old compilers
3. Fixed buffers exhaustion when compiled with statistics
4. Fixed runtime addition of Discovery Servers
5. Fixed dangling sample references with big data
6. Fixed history record issues with persistence
7. Correctly disable DataReader on destruction
8. Fixed alignment issues on XTypes QoS policies serialization
9. Fixed reconnection to Discovery Server
10. Correctly use builtin publisher for statistics DataWriters
11. Fixed various GCC-11 warnings
12. Use only public APIs from foonathan::memory
13. Fixed installation directories for DDS examples
14. Fixed read after free on security code

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id73"></a>

#### [Version 2.4.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.4.0/index.html)

This minor release is API compatible with the previous minor release, but introduces **ABI breaks** on
two of the three public APIs:

* Methods and attributes have been added on several classes of the DDS-PIM high-level API, so indexes of
  symbols on dynamic libraries may have changed.
* Methods and attributes have been added on several classes of the RTPS low-level API, so indexes of
  symbols on dynamic libraries may have changed.
* Old Fast-RTPS high-level API remains ABI compatible.

This minor release includes the following **features**:

* [Conditions and Wait-sets](../../fastdds/dds_layer/core/waitsets/waitsets.md#dds-layer-core-waitsets) implementation.
* [Flow controllers](../../fastdds/use_cases/large_data/large_data.md#flow-controllers).
* [Configure Discovery Server locators using names](../../fastdds/discovery/discovery_server.md#ds-dns-name).
* [Modifying remote servers list at run time](../../fastdds/discovery/discovery_server.md#ds-modify-server-list).
* [Environment file override](../../fastdds/env_vars/env_vars.md#env-vars-fastdds-environment-file).

It also includes the following **improvements**:

* Allow setting custom folder for [data-sharing](../../fastdds/transport/datasharing.md#datasharing-delivery) files.
* Allow setting persistence guid with static discovery.
* Check for NDEBUG in logInfo.
* Removed old unused CMake code.
* Fixed TLS behavior on TCP example.
* Prepare API for easy integration of python bindings.
* Improved statistics performance.

Some important **bugfixes** are also included:

* Fixed order of returned samples on topics with keys.
* Allow updating partitions to an empty set.
* Correctly propagate DomainParticipantQos updates.
* Avoid a volatile data-sharing reader to block a writer.
* Correctly give priority to intra-process over data-sharing.
* Fixed reallocation issue on LivelinessManager.
* Fixed deadline issue on volatile DataWriter
* Fixed STRICT_REALTIME silently not active with Unix POSIX systems.
* Fixed build errors with OpenSSL 3.0

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="version-2-3-eol"></a>

### Version 2.3 (EOL)

<a id="id74"></a>

#### [Version 2.3.6 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.3.6/index.html)

This release includes the following **improvements**:

1. Improve rediscovery on lossy environments.
2. Upgrade CMake minimum requirement to 3.13.
3. Improve `Guid_t` operator < performance.

This release includes the following **bugfixes**:

1. Fixed validation on `ParameterPropertyList_t`.
2. Add python3 dependency to package.xml.
3. Fix null references and uncaught exceptions on XML parser.
4. Install Statistics IDL file.
5. Fix data races when creating DataWriters.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id75"></a>

#### [Version 2.3.5 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.3.5/index.html)

This release includes the following **improvements**:

1. Fixed several flaky tests.
2. Use native inter-process on Windows.
3. Support for partitions on DataWriterQoS and DataReaderQoS.
4. Support for GCC 12.
5. Correctly implement delete_contained_entities.

This release also includes the following **bugfixes**:

1. Fixed deadline issue on volatile DataWriter.
2. Allow updating partitions to an empty set.
3. Fixed order of returned samples on topics with keys.
4. Fixed issues in LivelinessManager.
5. Correctly give priority to intra-process over data-sharing.
6. Avoid bad_node_size exception when cross-building.
7. Fixed build errors with OpenSSL 3.0.
8. Avoid a volatile data-sharing reader to block a writer.
9. Fixed history record issues with persistence.
10. Correctly disable DataReader on destruction.
11. Fixed various GCC 11 warnings.
12. Fixed payload pool handling on EDPSimple destructor.
13. Fixed read after free on security code.
14. Fixed null dereference on XML parser.
15. Ensure correct boost singleton destruction order.
16. Enable memory protection on DataSharing readers.
17. TCP reconnection issues.
18. MemberDescriptor fully qualified name.
19. Fix recommended statistics DataReaderQos to enable backwards compatibility.
20. Fixed dangling sample references with big data.
21. Fixed deadlocks and data races.
22. Fixed reconnection to Discovery Server.
23. Other minor fixes.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id76"></a>

#### [Version 2.3.4 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.3.4/index.html)

This release includes the following **improvements**:

1. Support of googletest using colcon
2. Network latency reports source participant
3. Update Fast DDS Gen to v2.0.2

This release includes the following **bugfixes**:

1. Fix mutex lock count on `PDPListener`
2. Limit `SequenceNumberSet` number of bits on deserialization
3. Fix segmentation fault on discovery server
4. Fix deadlock with security and timers
5. Fix bug using not protected code in a test
6. Fix deadlock with `LivelinessManager`
7. Fix interval loop on events
8. Fix run event when was cancelled
9. Validate sequence range on `CDRMessage::readSequenceNumberSet`
10. Fix subscription throughput data generation
11. Allow examples to build on QNX
12. Fix code on SHM clean
13. Accept Statistics DataWriters in Discovery Server
14. Fix read/take behavior when a future change is found
15. Correctly handle deserialization errors on `read_next_sample()` / `take_next_sample()`
16. Fixing `SequenceNumberSet_t` deserialization
17. Proper history clean up when a reader unmatches a writer
18. Unprotected code loaning samples
19. Fix publication throughput statistic on volatile writers
20. Fix Fast DDS CLI server name
21. Several fixes in examples and tests

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id77"></a>

#### [Version 2.3.3 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.3.3/index.html)

This release includes the following **improvements**:

* Added more durability kinds in Static Discovery xml parser
* Explicitly enable/disable data-sharing on performance tests
* Allow fully qualified name in TypeDescriptor
* Added missing DynamicData::get_union_id() method
* Change log severity in DiscoveryServer first announcement
* Several corrections to README

This release includes the following **bugfixes**:

* Fixed warnings and segfaults on 32-bit platforms
* Fixed UDPv6 behavior
* Fixed persistence guid issue on statistics writers
* Fixed static linking with open SSL
* Fixed statistics header file inclusion
* Fixed build on RedHat systems

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id78"></a>

#### [Version 2.3.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.3.2/index.html)

This release includes the following **feature**:

* Statistics Module

It also includes the following **improvements**:

* Update Asio submodule and avoid exporting Asio API
* Improve Windows installers
* Ease Google Fuzz integration
* Improve Doxygen documentation on lifetime of pointers created with `RTPSDomain`
* Update Fast CDR to v1.0.21

This release includes the following **bugfixes**:

* Add a correct multicast address for UDPv6
* Recover from out-of-sync TCP datagrams

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id79"></a>

#### [Version 2.3.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.3.1/index.html)

This release includes several **bugfixes** and **improvements**:

* Added [Fast DDS Statistics Module](../../fastdds/statistics/statistics.md#statistics) implementation
* Fixed alignment issues on generated code calculation of maximum serialized size
* Fixed calculation of data-sharing domain id
* Fixed issues on data-sharing with volatile writers
* Fixed build issues on old compilers
* Fixed some tests when the library is built without security
* Fixed and exposed pull mode on writers
* Fixed handling of –data_sharing on latency test
* Fixed calculation of memory pools sizes on debug builds
* Correctly update memory policy on writers and readers

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id80"></a>

#### [Version 2.3.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.3.0/index.html)

This minor release is API compatible with the previous minor release, but introduces **ABI breaks** on
two of the three public APIs:

* Methods and attributes have been added on several classes of the DDS-PIM high-level API, so indexes of
  symbols on dynamic libraries may have changed.
* Methods and attributes have been added on several classes of the RTPS low-level API, so indexes of
  symbols on dynamic libraries may have changed.
* Old Fast-RTPS high-level API remains ABI compatible.

This release adds the following **features**:

* [Unique network flows](../../fastdds/use_cases/unique_network_flows/unique_network_flows.md#use-case-unique-flows)
* [Discovery super-client](../../fastdds/ros2/discovery_server/ros2_discovery_server.md#ros2-discovery-server-introspection)
* [Statistics module API](../../fastdds/statistics/statistics.md#statistics)
* [New flow controller API](../../fastdds/api_reference/dds_pim/core/policy/publishmodeqospolicy.md#api-pim-publishmodeqospolicy)
* [Static discovery configuration from raw string](../../fastdds/discovery/static.md#static-xml-load)
* [Added reception timestamp to SampleInfo](../../fastdds/api_reference/dds_pim/subscriber/sampleinfo.md#api-pim-sampleinfo)
* [Exposing get_unread_count on DataReader](../../fastdds/api_reference/dds_pim/subscriber/datareader.md#api-pim-datareader)

It also includes the following **improvements**:

* Data-sharing delivery internal refactor
* Additional metadata on persistence databases
* Refactor on ReturnCode_t to make it switch friendly
* Performance tests refactored to use DDS-PIM high-level API
* Receive const pointers on delete_xxx methods
* Discovery server improvements
* Made SOVERSION follow major.minor

Some important **bugfixes** are also included:

* Fixed shared memory usage on QNX
* Fixed reference counting on internal pools
* Fixed singleton destruction order
* Fixed interoperability issues with x-types information
* Fixed recovery of shared memory buffers
* Lifespan support in persistent writers

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="version-2-2-eol"></a>

### Version 2.2 (EOL)

<a id="id81"></a>

#### [Version 2.2.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.2.1/index.html)

This release includes the following **improvements**:

1. Data-sharing delivery internal refactor.
2. Performance tests refactored to use DDS-PIM high-level API.
3. Discovery server improvements.

This release includes the following **bugfixes**:

1. Fixed reference counting on internal pools.
2. Fixed singleton destruction order.
3. Fixed default multicast locators.
4. Fixed interoperability issues with x-types information.
5. Fixed Reader history issues.
6. Fixed data races issues.
7. Fixed shared memory issues.
8. Fixed heartbeat and ACK issues.
9. Fixed LivelinessManager issues.
10. Fixed TCP reception synchronization.
11. Fixed build issues on old compilers.
12. Allow modifying Partition QoS in enabled entities.
13. Other minor fixes and improvements.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id82"></a>

#### [Version 2.2.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.2.0/index.html)

This minor release is API compatible with the previous minor release, but introduces **ABI breaks** on
two of the three public APIs:

* Methods and attributes have been added on several classes of the DDS-PIM high-level API, so indexes of
  symbols on dynamic libraries may have changed.
* Methods and attributes have been added on several classes of the RTPS low-level API, so indexes of
  symbols on dynamic libraries may have changed.
* Old Fast-RTPS high-level API remains ABI compatible.

This release adds the following **features**:

* Data Sharing delivery (avoids transport encapsulation for localhost communications)
* Complete DDS-PIM high-level API declarations
* Extension APIs allowing zero-copy delivery (both intra-process and inter-process)
* Upgrade to Quality Level 1

It also includes the following **improvements**:

* Code coverage policy
* Added several tests to increase coverage
* Increased GUID uniqueness
* Allow logInfo messages to be compiled on build types other than debug

Some important **bugfixes** are also included:

* Fixed timed events manager race condition
* Fixed payload protection issues with SHM transport
* Writers correctly handle infinite resource limits on keyed topics
* Fixed unsafe code on AESGCMGMAC plugin
* Several fixes for IPv6 (whitelists, address parser)
* Fixes on liveliness timing handling
* Fixed warnings building on C++20

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="version-2-1-eol"></a>

### Version 2.1 (EOL)

<a id="id83"></a>

#### [Version 2.1.4 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.1.4/index.html)

This release includes the following **improvements**:

1. Improve auto GAPs in Data Sharing
2. Improve behavior when `STRICT_REALTIME` [CMake option](../../installation/configuration/cmake_options.md#cmake-options) is not enabled
3. Handle SIGTERM in fast discovery server CLI

This release includes the following **bugfixes**:

1. Select correct listener for on_requested_deadline_missed
2. Correctly assign multicast port to multicast initial peers
3. Fix chain of trust issues with a single CA certificate
4. Correctly handle builtin endpoints mask
5. Fix build on MSVC 19

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from a version older than 1.10.0, regenerating the code is *recommended*.

<a id="id84"></a>

#### [Version 2.1.3 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.1.3/index.html)

This release includes the following **improvements**:

1. Improve rediscovery on lossy environments.

This release includes the following **bugfixes**:

1. Fixed validation on `ParameterPropertyList_t`.
2. Fixed `SequenceNumberSet_t` deserialization.
3. Add python3 dependency to package.xml.
4. Fix data races when creating DataWriters.
5. Fix deadlock when removing remote DomainParticipants by expired liveliness when using Security.
6. Fix communication with asymmetric [Ignore Participant flags](../../fastdds/discovery/general_disc_settings.md#discovery-ignore-flags).

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from a version older than 1.10.0, regenerating the code is *recommended*.

<a id="id85"></a>

#### [Version 2.1.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.1.2/index.html)

This release includes the following **improvements**:

1. Allow fully qualified name in TypeDescriptor.
2. Use native inter-process on Windows.
3. Support for GCC 12.
4. Support of googletest using colcon.

This release also includes the following **bugfixes**:

1. Fixed recovery of shared memory buffers.
2. Fixed issues in LivelinessManager.
3. Fixed default multicast locators.
4. Fixed TCP issues.
5. Fixed deadlocks and data races.
6. Fixed deadline issue on volatile DataWriter.
7. Avoid bad_node_size exception when cross-building.
8. Fixed order of returned samples on topics with keys.
9. Allow updating partitions to an empty set.
10. Suppress OpenSSL 3.0 warnings.
11. MemberDescriptor fully qualified name.
12. Fixed history record issues with persistence.
13. Fixed reconnection to Discovery Server.
14. Other minor fixes.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from a version older than 1.10.0, regenerating the code is *recommended*.

<a id="id86"></a>

#### [Version 2.1.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.1.1/index.html)

This release includes the following **bugfixes**:

* Fixed race condition on security handshake
* Fixed SHM data corruption when using both payload and sub-message protection
* Fixed some interoperability issues
* Fixed race condition on timed-events thread
* Fixed usage of SHM on QNX systems

It also includes the following **improvements**:

* Increased uniqueness of GUID prefix
* Discovery server improvements

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from a version older than 1.10.0, regenerating the code is *recommended*.

<a id="id87"></a>

#### [Version 2.1.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.1.0/index.html)

This minor release is API compatible with the previous minor release, but introduces **ABI breaks** on
two of the three public APIs:

* Methods and attributes have been added on several classes of the DDS-PIM high-level API, so indexes of
  symbols on dynamic libraries may have changed.
* Methods and attributes have been added on several classes of the RTPS low-level API, so indexes of
  symbols on dynamic libraries may have changed.
* Old Fast-RTPS high-level API remains ABI compatible.

Users of the RTPS low-level API should also be aware of the following **API deprecations**:

* History::reserve_Cache has been deprecated
  * Methods RTPSWriter::new_change or RTPSReader::reserveCache should be used instead
* History::release_Cache has been deprecated
  * Methods RTPSWriter::release_change or RTPSReader::releaseCache should be used instead

This release adds the following **features**:

* Support persistence for large data
* Added support for on_requested_incompatible_qos and on_offered_incompatible_qos
* SKIP_DEFAULT_XML environment variable
* Added FORCE value to THIRDPARTY cmake options
* New log consumer (StdOutErrConsumer)
* Added methods to get qos defined in XML Profile
* Support for persistence on TRANSIENT_LOCAL

It also includes the following **improvements**:

* Internal refactor for intra-process performance boost
* Allow usage of foonathan/memory library built without debug tool
* Large data support on performance tests
* Reduced flakiness of several tests

Some important **bugfixes** are also included:

* Fixed behavior of several DDS API methods
* Fixed interoperability issues with RTI connext
* Fixed DLL export of some methods
* Avoid redefinition of compiler defined macros
* Fixed some intra-process related segmentation faults and deadlocks
* Fixed large data payload protection issues on intra-process
* Fixed C++17 and VS 2019 warnings
* Fixed linker problems on some platforms
* Fixed transient local retransmission after participant drop
* Fixed assertion failure on persistent writers

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from a version older than 1.10.0, regenerating the code is *recommended*.

<a id="version-2-0-eol"></a>

### Version 2.0 (EOL)

<a id="id88"></a>

#### [Version 2.0.3 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.0.3/index.html)

It also includes the following **improvements**:

1. Increased uniqueness of GUID prefix (#1648)
2. Upgrade Fast CDR to v1.0.20 (#1793)

This release includes the following **bugfixes**:

1. Fixed some race conditions (#1540, #2023)
2. Fixed SHM issues (#1644, #1895, #2266)
3. Fixed some interoperability issues (#1624, #1752, #1849)
4. Fixed Discovery Server 2.0 issues (#1639, #1651, #1761, #1796)
5. Fixed several issues on QNX systems (#1744, #1773, #1776)
6. Fix singleton destruction order (#1758)
7. Fix heartbeat and ACK issues (#1865)
8. Fix issues in LivelinessManager (#1872, #2147)
9. Fix multicast issues (#1966, #1905)
10. Fix TCP reception synchronization (#1981)
11. XTypes standard compliance and fixes (#2006, #2278)
12. Other minor fixes (#1558, #1734, #1814, #1935, #1978, #2121)

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from a version older than 1.10.0, regenerating the code is *recommended*.

<a id="id89"></a>

#### [Version 2.0.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.0.2/index.html)

This release includes the following improvements:

* Improve QNX support
* Security improvements
* Fast DDS Quality Declaration (QL 2)
* Large traffic reduction when using Discovery Server (up to 85-90% for large deployments)
* Configuration of Clients of Discovery Server using an environment variable
* A CLI for Fast DDS:
  * This can be used to launch a discovery server
  * Clean SHM directories with one command
* Shared memory transport enabled by default
* Solved edge-case interoperability issue with CycloneDDS
* Add package.xml

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from a version older than 1.10.0, regenerating the code is *recommended*.

<a id="id90"></a>

#### [Version 2.0.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.0.1/index.html)

This release includes the following bug fixes:

* Fixed sending GAPs to late joiners
* Fixed asserting liveliness on data reception
* Avoid calling `OpenSSL_add_all_algorithms()` when not required

Other improvements:

* Fixing warnings

PRs in merge order:
[#1295](https://github.com/eProsima/Fast-DDS/pull/1295),
[#1300](https://github.com/eProsima/Fast-DDS/pull/1300),
[#1304](https://github.com/eProsima/Fast-DDS/pull/1304),
[#1290](https://github.com/eProsima/Fast-DDS/pull/1290),
[#1307](https://github.com/eProsima/Fast-DDS/pull/1307).

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from a version older than 1.10.0, regenerating the code is *recommended*.

<a id="id96"></a>

#### [Version 2.0.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v2.0.0/index.html)

This release has the following **API breaks**:

* eClock API, which was deprecated on v1.9.1, has been removed
* eprosima::fastrtps::rtps::RTPSDomain::createParticipant methods now have an additional first argument domain_id
* Data member domainId has been removed from eprosima::fastrtps::rtps::RTPSParticipantAttributes and added to
  eprosima::fastrtps::ParticipantAttributes

Users should also be aware of the following **deprecation announcement**:

* All classes inside the namespace eprosima::fastrtps should be considered deprecated.
  Equivalent functionality is offered through namespace eprosima::fastdds.
* Namespaces beneath eprosima::fastrtps are not included in this deprecation, i.e.
  eprosima::fastrtps::rtps can still be used)

This release adds the following **features**:

* Added support for register/unregister/dispose instance
* Added DDS compliant API. This new API exposes all the functionality of the Publisher-Subscriber Fast RTPS API
  adhering to the [Data Distribution Service (DDS) version 1.4 specification](https://www.omg.org/spec/DDS/1.4)
* Added Security Logging Plugin (contributed by Cannonical Ltd.)
* Bump to FastCDR v1.0.14

It also includes the following bug fixes and improvements:

* Support for OpenSSL 1.1.1d and higher
* Support for latest versions of gtest
* Support for FreeBSD
* Fault tolerance improvements to Shared Memory transport
* Fixed segfault when no network interfaces are detected
* Correctly ignoring length of PID_SENTINEL on parameter list
* Improved traffic on PDP simple mode
* Reduced CPU and memory usage

<a id="version-1-10-eol"></a>

### Version 1.10 (EOL)

<a id="id97"></a>

#### [Version 1.10.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.10.1/index.html)

This release includes the following **improvements**:

1. Add new CMake option: SHM_TRANSPORT_DEFAULT. Shared Memory (SHM) Transport disabled by default.
2. Comply with the RTPS standard concerning PID_SENTINEL.
3. Support for OpenSSL 1.1.1d.

This release includes the following bugfixes:

1. Fix crash when there are no network interfaces.
2. Several Shared Memory Transport fixes.
3. Other minor fixes.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*.
If you are upgrading from a version older than 1.10.0, regenerating the code is *recommended*.

<a id="id98"></a>

#### [Version 1.10.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.00.0/index.html)

This release adds the following features:

* New built-in [Shared Memory Transport](../../fastdds/transport/shared_memory/shared_memory.md#transport-sharedmemory-sharedmemory)
* Transport API refactored to support locator iterators
* Added subscriber API to retrieve info of first non-taken sample
* Added parameters to fully avoid dynamic allocations
* History of built-in endpoints can be configured
* Bump to FastCDR v1.0.13.
* Bump to Fast-RTPS-Gen v1.0.4.
* Require CMake 3.5 but use policies from 3.13

It also includes the following bug fixes and improvements:

* Fixed alignment on parameter lists
* Fixed error sending more than 256 fragments.
* Fix handling of STRICT_REALTIME.
* Fixed submessage_size calculation on last data_frag.
* Solved an issue when recreating a publishing participant with the same GUID.
* Solved an issue where a publisher could block on write for a long time when a new
  subscriber (late joiner) is matched, if the publisher had already sent a large number
  of messages.
* Correctly handling the case where lifespan expires at the same time on several samples.
* Solved some issues regarding liveliness on writers with no readers.
* Correctly removing changes from histories on keyed topics.
* Not reusing cache change when sample does not fit.
* Fixed custom wait_until methods when time is in the past.
* Several data races and ABBA locks fixed.
* Reduced CPU and memory usage.
* Reduced flakiness of liveliness tests.
* Allow for more use cases on performance tests.

Several bug fixes on discovery server:

* Fixed local host communications.
* Correctly trimming server history.
* Fixed backup server operation.
* Fixed timing issues.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*.
If you are upgrading from a version older than 1.10.0, regenerating the code is *recommended*.

<a id="version-1-9-eol"></a>

### Version 1.9 (EOL)

<a id="id99"></a>

#### [Version 1.9.5 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.9.5/index.html)

This release includes the following **improvements**:

1. Propagate serialization error when reading samples from Subscriber History.
2. Improvements in test suite.
3. Improvements to reduce memory consumption.
4. Update CMake (3.5) using newer policies.
5. Improve PDP StatefulWriter announcement.
6. Performance improvements.
7. Message receiver improvements.
8. QoS Policies improvements.

This release includes the following bugfixes:

1. Fix compiler warnings in Windows when building the test suite.
2. Fix several data races.
3. Fix History issues.
4. Fix errors when sending data fragments.
5. Fix strict real-time behavior.
6. Fix in Discovery Server.
7. Fix CMake option.
8. Fix interoperability issues.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*.

<a id="id100"></a>

#### [Version 1.9.4 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.9.4/index.html)

This release adds the following features:

* Intra-process delivery mechanism is now active by default.
* Synchronous writers are now allowed to send fragments.
* New memory mode DYNAMIC_RESERVE on history pool.
* Performance tests can now be run on Windows and Mac.
* XML profiles for requester and replier.

It also includes the following bug fixes and improvements:

* Bump to FastCDR v1.0.12.
* Bump to Fast-RTPS-Gen v1.0.3.
* Fixed deadlock between PDP and StatefulReader.
* Improved CPU usage and allocations on timed events management.
* Performance improvements on reliable writers.
* Fixing bugs when Intra-process delivery is activated.
* Reducing dynamic allocations and memory footprint.
* Improvements and fixes on performance tests.
* Other minor bug fixes and improvements.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*.

<a id="id101"></a>

#### [Version 1.9.3 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.9.3/index.html)

This release adds the following features:

* Participant discovery filtering flags.
* Intra-process delivery mechanism opt-in.

It also includes the following bug fixes and improvements:

* Bump to Fast-RTPS-Gen v1.0.2.
* Bring back compatibility with XTypes 1.1 on PID_TYPE_CONSISTENCY.
* Ensure correct alignment when reading a parameter list.
* Add CHECK_DOCUMENTATION *cmake* option.
* EntityId_t and GuidPrefix_t have now their own header files.
* Fix potential race conditions and deadlocks.
* Improve the case where *check_acked_status* is called between reader matching process and its acknack reception.
* RTPSMessageGroup_t instances now use the thread-local storage.
* FragmentedChangePitStop manager removed.
* Remove the data fragments vector on CacheChange_t.
* Only call find_package for TinyXML2 if third-party options are off
* Allow XMLProfileManager methods to not show error log messages if a profile is not found.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*.

<a id="id102"></a>

#### [Version 1.9.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.9.2/index.html)

This release includes the following feature:

* Multiple initial PDP announcements.
* Flag to avoid builtin multicast.

It also adds the following bug fixes and improvements:

* Bump to Fast-RTPS-Gen v1.0.1.
* Bump to IDL-Parser v1.0.1.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*.

<a id="id103"></a>

#### [Version 1.9.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.9.1/index.html)

This release includes the following features:

* Fast-RTPS-Gen is now an independent project.
* Header **eClock.h** is now marked as deprecated.

It also adds the following bug fixes and improvements:

* Bump to FastCDR v1.0.11.
* Installation from sources documentation fixed.
* Fixed assertion on WriterProxy.
* Fixed potential fall through while parsing Parameters.
* Removed deprecated guards causing compilation errors in some 32 bits platforms.
* *addTOCDRMessage* method is now exported in the DLL, fixing issues related with Parameters’ constructors.
* Improve windows performance by avoiding usage of  *\_Cnd_timedwait* method.
* Fixed reported communication issues by sending multicast through *localhost* too.
* Fixed potential race conditions and deadlocks.
* Eliminating use of *acceptMsgDirectTo*.
* Discovery Server framework reconnect/recreate strategy.
* Removed unused folders.
* Restored subscriber API.
* SequenceNumber_t improvements.
* Added STRICT_REALTIME *cmake* option.
* SubscriberHistory improvements.
* Assertion of participant liveliness by receiving RTPS messages from the remote participant.
* Fixed error while setting next deadline event in *create_new_change_with_params*.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*.

<a id="id104"></a>

#### [Version 1.9.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.9.0/index.html)

This release includes the following features:

* Partial implementation of allocation QoS.
* Implementation of Discovery Server.
* Implementation of non-blocking calls.

It also adds the following bug fixes and improvements:

* Added sliding window to BitmapRange.
* Modified default behavior for unknown writers.
* A Flush() method was added to the logger to ensure all info is logged.
* A test for loading Duration_t from XML was added.
* Optimized WLP when removing local writers.
* Some liveliness tests were updated so that they are more stable on Windows.
* A fix was added to CMakeLists.txt for installing static libraries.
* A fix was added to performance tests so that they can run on the RT kernel.
* Fix for race condition on built-in protocols creation.
* Fix for setting *nullptr* in a *fixed_string*.
* Fix for v1.8.1 not building with -DBUILD_JAVA=ON.
* Fix for GAP messages not being sent in some cases.
* Fix for coverity report.
* Several memory issues fixes.
* fastrtps.repos file was updated.
* Documentation for building with Colcon was added.
* Change CMake configuration directory if INSTALLER_PLATFORM is set.
* IDL sub-module updated to current version.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*.

<a id="version-1-8-eol"></a>

### Version 1.8 (EOL)

<a id="id105"></a>

#### [Version 1.8.5 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.8.5/index.html)

This release includes the following **bugfixes**:

1. Fix Subscriber History to correctly notify late-joiners in case of KEEP_LAST, RELIABLE, and TRANSIENT_LOCAL.
2. Fix Writer History behavior when there are no matched readers.
3. Fix heartbeat and ACK issues.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*.

<a id="id106"></a>

#### [Version 1.8.4 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.8.4/index.html)

This release adds the following **feature**:

* XML profiles for requester and replier

It also has the following **important bug fixes**:

* Solved an issue when recreating a publishing participant with the same GUID (either on purpose or by chance)
* Solved an issue where a publisher could block on write for a long time when, after a large number of samples
  have been sent, a new subscriber is matched.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*

<a id="id107"></a>

#### [Version 1.8.3 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.8.3/index.html)

This release adds the following bug fixes and improvements:

* Fix serialization of TypeConsistencyEnforcementQosPolicy.
* Bump to Fast-RTPS-Gen v1.0.2.
* Bump to IDL-Parser v1.0.1.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*

<a id="id108"></a>

#### [Version 1.8.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.8.2/index.html)

This release includes the following features:

* Modified unknown writers default behavior.
* Multiple initial PDP announcements.
* Flag to avoid builtin multicast.
* *STRICT_REALTIME* compilation flag.

It also adds the following bug fixes and improvements:

* Fix for setting nullptr in a fixed string.
* Fix for not sending GAP in several cases.
* Solve *Coverity* report issues.
* Fix issue of *fastddsgen* failing to open *IDL.g4* file.
* Fix unnamed lock in *AESGCMGMAC_KeyFactory.cpp*.
* Improve *XMLProfiles* example.
* Multicast is now sent through *localhost* too.
* *BitmapRange* now implements sliding window.
* Improve *SequenceNumber_t* struct.
* Participant’s liveliness is now asserted when receiving any RTPS message.
* Fix leak on RemoteParticipantLeaseDuration.
* Modified default values to improve behavior in *Wi-Fi* scenarios.
* *SubscriberHistory* improvements.
* Removed use of *acceptMsgDirectTo*.
* *WLP* improvements.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*

<a id="id109"></a>

#### [Version 1.8.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.8.1/index.html)

This release includes the following features:

* Implementation of [LivelinessQosPolicy](../../fastdds/dds_layer/core/policy/standardQosPolicies.md#livelinessqospolicy) QoS.

It also adds the following bug fixes and improvements:

* Fix for get_change on history, which was causing issues during discovery.
* Fix for announcement of participant state, which was sending ParticipantBuiltinData twice.
* Fix for closing multicast UDP channel.
* Fix for race conditions in SubscriberHistory, UDPTransportInterface and StatefulReader.
* Fix for lroundl error on Windows in Time_t.
* CDR & IDL submodules update.
* Use of java 1.8 or greater for fastddsgen.jar generation.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*.

<a id="id110"></a>

#### [Version 1.8.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.8.0/index.html)

This release included the following features:

* Implementation of IDL 4.2.
* Implementation of [DeadlineQosPolicy](../../fastdds/dds_layer/core/policy/standardQosPolicies.md#deadlineqospolicy) QoS.
* Implementation of [LifespanQosPolicy](../../fastdds/dds_layer/core/policy/standardQosPolicies.md#lifespanqospolicy) QoS.
* Implementation of [DisablePositiveACKsQosPolicy](../../fastdds/dds_layer/core/policy/eprosimaExtensions.md#disablepositiveacksqospolicy) QoS.
* Secure sockets on TCP transport ([TLS over TCP](../../fastdds/transport/tcp/tls.md#transport-tcp-tls)).

It also adds the following improvements and bug fixes:

* Real-time improvements: non-blocking write calls for best-effort writers, addition of fixed size strings,
  fixed size bitmaps, resource limited vectors, etc.
* Duration parameters now use nanoseconds.
* Configuration of participant mutation tries.
* Automatic calculation of the port when a value of 0 is received on the endpoint custom locators.
* Non-local addresses are now filtered from whitelists.
* Optimization of check for acked status for stateful writers.
* Linked libs are now not exposed when the target is a shared lib.
* Limitation on the domain ID has been added.
* UDP non-blocking send is now optional and configurable via XML.
* Fix for non-deterministic tests.
* Fix for ReaderProxy history being reloaded incorrectly in some cases.
* Fix for RTPS domain hostid being potentially not unique.
* Fix for participants with different lease expiration times failing to reconnect.

**Known issues**

* When using TPC transport, sometimes callbacks are not invoked when removing a participant due to a bug in ASIO.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*.

<a id="version-1-7-eol"></a>

### Version 1.7 (EOL)

<a id="id111"></a>

#### [Version 1.7.3 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.7.3/index.html)

This release includes the following **bugfixes**:

1. Remove inline specifier from public method not defined in header file.
2. Fix FastRTPS-Gen version generation

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*.

<a id="id112"></a>

#### [Version 1.7.2 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.7.2/index.html)

This release fixes an important bug:

* Allocation limits on subscribers with a KEEP_LAST QoS was taken from resource limits configuration
  and didn’t take history depth into account.

It also has the following improvements:

* Vendor FindThreads.cmake from CMake 3.14 release candidate to help with sanitizers.
* Fixed format of gradle file.

Some other minor bugs and performance improvements.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*.

<a id="id113"></a>

#### [Version 1.7.1 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.7.1/index.html)

This release included the following features:

* LogFileConsumer added to the logging system.
* Handle FASTRTPS_DEFAULT_PROFILES_FILE environment variable indicating the default profiles XML file.
* XML parser made more restrictive and with better error messages.

It also fixes some important bugs:
\* Fixed discovery issues related to the selected network interfaces on Windows.
\* Improved discovery times.
\* Workaround ASIO issue with multicast on QNX systems.
\* Improved TCP transport performance.
\* Improved handling of key-only data submessages.

Some other minor bugs and performance improvements.

**KNOWN ISSUES**

* Allocation limits on subscribers with a KEEP_LAST QoS is taken from resource limits configuration
  and doesn’t take history depth into account.

**Note:** If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source
from IDL files using *fastddsgen*.

<a id="id114"></a>

#### [Version 1.7.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.7.0/index.html)

This release included the following features:

* [TCP Transport](../../fastdds/transport/tcp/tcp.md#transport-tcp-tcp).
* [XTypes](../../fastdds/xtypes/xtypes.md#dynamic-types).
* Security 1.1 compliance.

Also bug fixing, allocation and performance improvements.

**Note:** If you are upgrading from an older version, it is **required** to regenerate generated source from IDL files
using *fastddsgen*.

<a id="version-1-6-eol"></a>

### Version 1.6 (EOL)

<a id="id115"></a>

#### [Version 1.6.0 (EOL)](https://fast-dds.docs.eprosima.com/en/v1.6.0/index.html)

This release included the following features:

* Persistence.
* Security access control plugin API and builtin [Access control plugin: DDS:Access:Permissions](../../fastdds/security/access_control_plugin/access_control_plugin.md#access-permissions) plugin.

Also bug fixing.

**Note:** If you are upgrading from an older version than 1.4.0, it is advisable to regenerate generated source from IDL
files using *fastddsgen*.

<a id="version-1-5-eol"></a>

### Version 1.5 (EOL)

<a id="version-1-5-0-eol"></a>

#### Version 1.5.0 (EOL)

This release included the following features:

* Configuration of Fast RTPS entities through XML profiles.
* Added heartbeat piggyback support.

Also bug fixing.

**Note:** If you are upgrading from an older version than 1.4.0, it is advisable to regenerate generated source from IDL
files using *fastddsgen*.

<a id="version-1-4-eol"></a>

### Version 1.4 (EOL)

<a id="version-1-4-0-eol"></a>

#### Version 1.4.0 (EOL)

This release included the following:

* Added secure communications.
* Removed all Boost dependencies. Fast RTPS is not using Boost libraries anymore.
* Added compatibility with Android.
* Bug fixing.

**Note:** After upgrading to this release, it is advisable to regenerate generated source from IDL files using
*fastddsgen*.

<a id="version-1-3-eol"></a>

### Version 1.3 (EOL)

<a id="version-1-3-1-eol"></a>

#### Version 1.3.1 (EOL)

This release included the following:

* New examples that illustrate how to tweak Fast RTPS towards different applications.
* Improved support for embedded Linux.
* Bug fixing.

<a id="version-1-3-0-eol"></a>

#### Version 1.3.0 (EOL)

This release introduced several new features:

* Unbound Arrays support: Now you can send variable size data arrays.
* Extended Fragmentation Configuration: It allows you to setup a Message/Fragment max size different to the standard
  64Kb limit.
* Improved logging system: Get even more introspection about the status of your communications system.
* Static Discovery: Use XML to map your network and keep discovery traffic to a minimum.
* Stability and performance improvements: A new iteration of our built-in performance tests will make benchmarking
  easier for you.
* ReadTheDocs Support: We improved our documentation format and now our installation and user manuals are available
  online on ReadTheDocs.

<a id="version-1-2-eol"></a>

### Version 1.2 (EOL)

<a id="version-1-2-0-eol"></a>

#### Version 1.2.0 (EOL)

This release introduced two important new features:

* Flow Controllers: A mechanism to control how you use the available bandwidth avoiding data bursts.
  The controllers allow you to specify the maximum amount of data to be sent in a specific period of time.
  This is very useful when you are sending large messages requiring fragmentation.
* Discovery Listeners: Now the user can subscribe to the discovery information to know the entities present in the
  network (Topics, Publishers & Subscribers) dynamically without prior knowledge of the system.
  This enables the creation of generic tools to inspect your system.

But there is more:

* Full ROS 2 Support: Fast RTPS is used by ROS 2, the upcoming release of the Robot Operating System (ROS).
* Better documentation: More content and examples.
* Improved performance.
* Bug fixing.
