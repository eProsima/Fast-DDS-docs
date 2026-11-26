<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="supported-versions"></a>

# Supported versions

<a id="fast-dds-pro"></a>

## Fast DDS **[Pro]**

<a id="version-3-7"></a>

### Version 3.7

<a id="release-notes-v3-7-0-1"></a>

<a id="version-3-7-0-1-pro"></a>

#### Version 3.7.0.1 **[Pro]**

This release includes the following **features**:

1. Add optional support on DynamicTypes API
2. Support truncating large collections in json_serialize
3. Make type support context inherit from Cdr context
4. Support member id in idl parser
5. Bring `Buffer<T>` from rosidl
6. Use machine ID in SHM locators

This release includes the following **improvements**:

1. Require all security plugins to be configured
2. Update fastcdr submodule to point to v2.4.0
3. Renaming Fast DDS Monitor to DDS Monitor

This release includes the following **security fixes (CVEs)**:

1. [CVE-2025-67108](https://www.cve.org/CVERecord?id=CVE-2025-67108)

This release includes the following **fixes**:

1. Fix volatile reader desync after initial positioning GAP
2. Fix FlowController not properly updating reader locators after reaching maximum limitation
3. Catch exception when constructing a payload pool Node
4. Disable Filewatch in QNX
5. Fix compilation error with `-DBUILD_SHARED_LIBS=OFF`
6. Fix datasharing infinite loop
7. Fix deadlock in the monitor service when a network interface changes
8. Account for padding when protecting payloads with GMAC only
9. Fix IDL parsing minor failures
10. Fix integer overflow in PartitionQosPolicy CDR deserialization

<a id="version-3-6"></a>

### Version 3.6

<a id="release-notes-v3-6-2-2"></a>

<a id="version-3-6-2-2-pro"></a>

#### Version 3.6.2.2 **[Pro]**

This release includes the following **security fixes**:

- [Fix GHSA-w92j-25c7-68qx](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-w92j-25c7-68qx)
- [Fix GHSA-33xc-m88x-3cp8](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-33xc-m88x-3cp8)
- [Fix GHSA-f5r6-qvhq-mx93](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-f5r6-qvhq-mx93)
- [Fix GHSA-wfcj-cv96-9wj9](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-wfcj-cv96-9wj9)
- [Fix GHSA-g355-8h7f-6w8v](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-g355-8h7f-6w8v)
- [Fix GHSA-6pwr-vh77-2x8m](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-6pwr-vh77-2x8m)
- [Fix GHSA-58x7-8g6g-mpp8](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-58x7-8g6g-mpp8)
- [Fix GHSA-f6cw-h682-xw3p](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-f6cw-h682-xw3p)
- [Fix GHSA-mg54-9fm8-xvf3](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-mg54-9fm8-xvf3)
- [Fix GHSA-j4xc-fjx8-wfg4](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-j4xc-fjx8-wfg4)
- [Fix GHSA-57rp-jm6r-xjw5](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-57rp-jm6r-xjw5)
- [Fix GHSA-f4hw-9j9g-j8xj](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-f4hw-9j9g-j8xj)
- [Fix GHSA-6m2p-wjv5-386v](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-6m2p-wjv5-386v)
- [Fix GHSA-5fgv-8wxq-3f69](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-5fgv-8wxq-3f69)
- [Fix GHSA-4r2x-mjm7-r285](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-4r2x-mjm7-r285)
- [Fix GHSA-vx4x-38rc-27w8](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-vx4x-38rc-27w8)
- [Fix GHSA-w75r-338m-pc5x](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-w75r-338m-pc5x)

This release includes the following **fixes**:

1. Fix volatile reader desynchronization after initial positioning GAP and `FlowController` not updating
   reader locators after reaching maximum limitation
2. Fix verification for Identity Certificate Validation
3. Tests and fixes for type support context-aware operations
4. Account for padding when protecting payloads with GMAC only
5. Fix datasharing infinite loop
6. Catch exception when constructing a payload pool Node
7. Fix integer overflow in `PartitionQosPolicy` CDR deserialization
8. Fix deadlock when toggling the internet interface that stops the monitor functioning
9. Improve validation of EDP static properties
10. Solve potential deadlock in `GrainedFlowController`
11. Fix leak in UDP when opening input channels
12. Set max ASIO buffers based on `IOV_MAX`
13. Select `IPv6` multicast interface by index instead of scope id
14. Fix minor data races
15. Fix default history depth in examples
16. Avoid RTPS statistics submessage mismatch
17. Fix `IPMonitor` shutdown delay
18. Fix IDL parsing minor failures

This release includes the following **improvements**:

1. Reduce extra send in Data(p) traffic for discovery server
2. Support member id in IDL parser
3. Fix IDL generation script
4. Support Bitmask on IDL parser
5. Remove Read the Docs related `CMake` instructions
6. Regenerate types with Fast DDS Gen 4.3.1.1
7. Update Fast CDR submodule to v2.3.7

This release includes the following **CI improvements**:

1. Single test for `DDSSQLFilterTests`
2. Fix CI docker access
3. Fix flaky tests
4. Delete ‘repos’ directory in ABI checker
5. Fix SEGV in latency test
6. Fix execution of profiling tests alternative build
7. Fix pass of `use-ccache` when running with dispatch
8. Bring Discovery Server testing framework to Fast DDS Pro
9. Improve `UBSan` and `ASan` CI jobs: merge jobs and not generate artifacts
10. Add `TSan` workflow to Fast DDS Pro
11. Fix deduced Gen branch

<a id="version-3-6-2-1-pro"></a>

#### Version 3.6.2.1 **[Pro]**

This release includes the following **security fixes (CVEs)**:

1. [CVE-2025-63829](https://www.cve.org/CVERecord?id=CVE-2025-63829)
2. [CVE-2025-65865](https://www.cve.org/CVERecord?id=CVE-2025-65865)
3. [CVE-2026-45092](https://www.cve.org/CVERecord?id=CVE-2026-45092)
4. [CVE-2026-45093](https://www.cve.org/CVERecord?id=CVE-2026-45093)
5. [CVE-2026-45094](https://www.cve.org/CVERecord?id=CVE-2026-45094)
6. [CVE-2026-45095](https://www.cve.org/CVERecord?id=CVE-2026-45095)
7. [CVE-2026-45096](https://www.cve.org/CVERecord?id=CVE-2026-45096)
8. [CVE-2026-45097](https://www.cve.org/CVERecord?id=CVE-2026-45097)
9. [CVE-2026-45098](https://www.cve.org/CVERecord?id=CVE-2026-45098)
10. [CVE-2026-49861](https://www.cve.org/CVERecord?id=CVE-2026-49861)
11. [CVE-2026-49862](https://www.cve.org/CVERecord?id=CVE-2026-49862)
12. [CVE-2026-49863](https://www.cve.org/CVERecord?id=CVE-2026-49863)
13. [CVE-2026-53588](https://www.cve.org/CVERecord?id=CVE-2026-53588)
14. [CVE-2026-53589](https://www.cve.org/CVERecord?id=CVE-2026-53589)
15. [CVE-2026-53590](https://www.cve.org/CVERecord?id=CVE-2026-53590)
16. [CVE-2026-55063](https://www.cve.org/CVERecord?id=CVE-2026-55063)

This release includes the following **fixes**:

1. Fix failure setting default locator in empty multicast locator list
2. Make `on_xxx_matched` callbacks thread-safe
3. Fix `RTPSParticipantAttributes` internal data races
4. Fix TCP reconnection with listening port
5. Fix QCC 12 compilation on ARM
6. Fix unused variable on `GCC 16`
7. Fix null-dereference build
8. Fix crash in the security handshake
9. Remove stale unbound channel resources
10. Fix `TypeObjectRegistry` null pass to a callee
11. Assert liveliness with periodic heartbeats in secure participants
12. Make `DataWriter`’s `set_qos` and `enable` methods thread-safe

This release includes the following **improvements**:

1. Writer-side content filtering on late joiners
2. Improve search of `ZLIB` and `BZip2`

<a id="version-3-6-1-1-pro"></a>

#### Version 3.6.1.1 **[Pro]**

This release includes the following **fixes**:

1. Apply `sqlite3` patch to eliminate const issue
2. Fix compilation warnings and errors in Ubuntu Focal

<a id="version-3-6-0-1-pro"></a>

#### Version 3.6.0.1 **[Pro]**

This release includes the following **security fixes (CVEs)**:

1. [CVE-2026-22591](https://www.cve.org/CVERecord?id=CVE-2026-22591)

This release includes the following **fixes**:

1. Fixed acknowledgement tracking in reliable DataWriters
2. Added SWIG macro to fix Fast DDS Python build

This release includes the following **improvements**:

1. TCP Keep Alive implementation
2. Add `should_endpoints_match_callback` in `DomainParticipant` and `RTPSParticipant`
3. Add context to type support operations
4. Limit the discovery announcement rate of Discovery Servers
5. Improve performance in `GUID`, `GUIDPrefix` and `EntityId` comparison operators

<a id="fast-dds"></a>

## Fast DDS

<a id="id1"></a>

### Version 3.6

<a id="id2"></a>

#### [Version 3.6.2](https://fast-dds.docs.eprosima.com/en/v3.6.2/index.html)

This release includes the following **security fixes (CVEs)**:

1. [CVE-2025-63829](https://www.cve.org/CVERecord?id=CVE-2025-63829)
2. [CVE-2025-65865](https://www.cve.org/CVERecord?id=CVE-2025-65865)
3. [CVE-2026-45092](https://www.cve.org/CVERecord?id=CVE-2026-45092)
4. [CVE-2026-45093](https://www.cve.org/CVERecord?id=CVE-2026-45093)
5. [CVE-2026-45094](https://www.cve.org/CVERecord?id=CVE-2026-45094)
6. [CVE-2026-45095](https://www.cve.org/CVERecord?id=CVE-2026-45095)
7. [CVE-2026-45096](https://www.cve.org/CVERecord?id=CVE-2026-45096)
8. [CVE-2026-45097](https://www.cve.org/CVERecord?id=CVE-2026-45097)

This release includes the following **fixes**:

1. Make `on_xxx_matched` thread-safe
2. Fix `RTPSParticipantAttributes` internal data races
3. Fix TCP reconnection with listening port
4. Fix failure setting default locator in empty multicast locator list
5. Fix unused variable on `GCC 16`
6. Fix null-dereference build
7. Fix dereference error of `shared_secret_handle`
8. Assert liveliness with periodic heartbeats in secure participants
9. Revert ABI extension to protect `UBSan` error
10. Fix data races on `DataWriterImpl` qos access
11. Fix TypeObjectRegistry null pass to a callee
12. Remove stale TCP channels

This release includes the following **improvements**:

1. Lower minimum CMake version in examples
2. Set Fast DDS version `3.5` as EOL
3. Changes after fastcdr branch-out to `2.4.x`
4. Mark `3.4.x` as EOL

This release includes the following **CI improvements**:

1. Improve windows tests
2. Rearrange `ASAN` workflow
3. Update security tests to use governance-based encryption configuration
4. Remove deprecated inputs from nightly
5. Add `UBSan` workflow and solve its errors
6. Make `skip-ci` label report required statuses as pass

<a id="id3"></a>

#### [Version 3.6.1](https://fast-dds.docs.eprosima.com/en/v3.6.1/index.html)

This release includes the following **fixes**:

1. Apply `sqlite3` patch to eliminate `const` issue
2. Remove deprecated doxygen warnings
3. Fix compilation warnings and errors in Ubuntu Focal

<a id="id4"></a>

#### [Version 3.6.0](https://fast-dds.docs.eprosima.com/en/v3.6.0/index.html)

This release includes the following **security fixes**:

1. [CVE-2026-22591](https://www.cve.org/CVERecord?id=CVE-2026-22591)

This release includes the following **improvements**:

1. Update `RELEASE_SUPPORT.md` with `2.14.6`
2. Add should_endpoints_match callback in `DomainParticipant` and `RTPSParticipant`
3. Add context to type support operations

This release includes the following **fixes**:

1. Change default python branch from `main` to `master`
2. Fix ABI break in `register_service_type`
3. Fix `TinyXML2` library resolution in tests

<a id="version-3-2"></a>

### Version 3.2

<a id="id5"></a>

#### [Version 3.2.5](https://fast-dds.docs.eprosima.com/en/v3.2.5/index.html)

This release includes the following **security fixes**:

- [Fix CVE-2025-63829](https://www.cve.org/CVERecord?id=CVE-2025-63829)
- [Fix CVE-2025-65865](https://www.cve.org/CVERecord?id=CVE-2025-65865)
- [Fix CVE-2025-67108](https://www.cve.org/CVERecord?id=CVE-2025-67108)
- [Fix CVE-2026-22591](https://www.cve.org/CVERecord?id=CVE-2026-22591)
- [Fix CVE-2026-45092](https://www.cve.org/CVERecord?id=CVE-2026-45092)
- [Fix CVE-2026-45093](https://www.cve.org/CVERecord?id=CVE-2026-45093)
- [Fix CVE-2026-45094](https://www.cve.org/CVERecord?id=CVE-2026-45094)
- [Fix CVE-2026-45095](https://www.cve.org/CVERecord?id=CVE-2026-45095)
- [Fix CVE-2026-45096](https://www.cve.org/CVERecord?id=CVE-2026-45096)
- [Fix CVE-2026-45097](https://www.cve.org/CVERecord?id=CVE-2026-45097)
- [Fix CVE-2026-45098](https://www.cve.org/CVERecord?id=CVE-2026-45098)
- [Fix CVE-2026-49861](https://www.cve.org/CVERecord?id=CVE-2026-49861)
- [Fix CVE-2026-49862](https://www.cve.org/CVERecord?id=CVE-2026-49862)
- [Fix CVE-2026-49863](https://www.cve.org/CVERecord?id=CVE-2026-49863)
- [Fix CVE-2026-53588](https://www.cve.org/CVERecord?id=CVE-2026-53588)
- [Fix CVE-2026-53589](https://www.cve.org/CVERecord?id=CVE-2026-53589)
- [Fix CVE-2026-53590](https://www.cve.org/CVERecord?id=CVE-2026-53590)
- [Fix CVE-2026-55063](https://www.cve.org/CVERecord?id=CVE-2026-55063)

This release includes the following **improvements**:

1. Update RELEASE_SUPPORT.md to only have link to master version
2. Fix several warnings on several GCC versions

This release includes the following **fixes**:

1. Fix `RTPSParticipantAttributes` internal data races
2. Catch exception when constructing a payload pool Node
3. Fix null-dereference build
4. Implement payload release in move assignment
5. Make `on_xxx_matched` thread-safe
6. Fix dereference error of ‘shared_secret_handle’
7. Assert liveliness with periodic heartbeats in secure participants
8. Fix data races on `DataWriterImpl` qos access
9. Remove stale TCP channels

This release includes the following **CI improvements**:

1. Make `skip-ci` label report required statuses as pass
2. Improve windows tests
3. Update security tests to use governance-based encryption configuration
4. Add `UBSan` workflow and solve its errors

<a id="id6"></a>

#### [Version 3.2.4](https://fast-dds.docs.eprosima.com/en/v3.2.4/index.html)

This patch release includes the following **security fixes**:

- [Fix CVE-2025-62599](https://www.cve.org/CVERecord?id=CVE-2025-62599)
- [Fix CVE-2025-62600](https://www.cve.org/CVERecord?id=CVE-2025-62600)
- [Fix CVE-2025-62601](https://www.cve.org/CVERecord?id=CVE-2025-62601)
- [Fix CVE-2025-62602](https://www.cve.org/CVERecord?id=CVE-2025-62602)
- [Fix CVE-2025-62603](https://www.cve.org/CVERecord?id=CVE-2025-62603)
- [Fix CVE-2025-62799](https://www.cve.org/CVERecord?id=CVE-2025-62799)
- [Fix CVE-2025-64098](https://www.cve.org/CVERecord?id=CVE-2025-64098)
- [Fix CVE-2025-64438](https://www.cve.org/CVERecord?id=CVE-2025-64438)
- [Fix CVE-2025-65016](https://www.cve.org/CVERecord?id=CVE-2025-65016)
- [Fix CVE-2026-22590](https://www.cve.org/CVERecord?id=CVE-2026-22590)

This patch release includes the following **improvements**:

1. Protect DataReaderInstance `alive_writers` attribute
2. Avoid casting `GUID_t` into `InstanceHandle_t`
3. Replace `logWarning` with `EPROSIMA_LOG_WARNING`
4. Improve some tests
5. Improvements in message receiver
6. Improvements on `compute_key` generated code
7. Verify Safe DDS signature
8. Improve Open SSL includes
9. Run `xtypes` test cases in parallel
10. Protect PDPStatelessWriter attributes
11. Allow to send empty payloads in dispose/unregister operations
12. Fix links to external documentation
13. Update to Fast CDR v2.3.5
14. Regenerate types with Fast DDS Gen 4.0.6

This patch release includes the following **fixes**:

1. Fix usage of invalid iterator in dynamic types
2. Fix DataReader history enforcement to respect `max_samples_per_instance`
3. Fix wrong update of default locators
4. Fix some build issues on non-tier 1 platforms
5. Fix GAP messages are not sent when there is no Reader requesting the DATA

This release includes the following **CI and release management updates**:

1. Allow CI to run tests with manual dispatch
2. Reduce disk usage in TSAN job
3. Upgrade to macOS Sequoia
4. Single line for `ctest_args` in Mac CI
5. Free disk space in Ubuntu runners
6. Upgrade `Linters` CI runner image to `ubuntu-24.04`

#### IMPORTANT
When upgrading to version 3.2.4 it is **highly recommended** to regenerate generated source from IDL files
using at least [Fast DDS-Gen v4.0.6](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.0.6).
But it is advisable to regenerate them using the latest patch version of `Fast DDS-Gen v4.0.x`.

<a id="id7"></a>

#### [Version 3.2.3](https://fast-dds.docs.eprosima.com/en/v3.2.3/index.html)

This release includes the following **features**:

1. Send only non-default QoS & Use `serialize_optional_qos` for optional QoS in EDP messages
2. Add `WireProtocolConfigQos` to `optionals` serialization and make `Monitor Service` always serialize
   `optionals`
3. RPC over DDS example
4. Bump to asio 1.34.2
5. Support aliases in Content Filtered Topic
6. Add methods for TypeObject registration of RPC types

This release includes the following **improvements**:

1. Add regression test checking type_information is always copied in `ProxyDatas` copies
2. Address some compilation warnings with latest `GCC`
3. Improvements on RPC entities
4. Add references to documentation
5. Update ROSin image links in `Readme`
6. Avoid calling `setIPv4` in `TCPTransportInterface`
7. Send heartbeat when [`RTPSWriter::wait_for_all_acked`](../../fastdds/api_reference/rtps/writer/RTPSWriter.md#_CPPv4N8eprosima7fastdds4rtps10RTPSWriter18wait_for_all_ackedERKN3dds10Duration_tE) is called
8. Use binary search in `find_change_nts`
9. Improve latency tests
10. Apply `uncrustify` to all files

This release includes the following **fixes**:

1. Update reception timestamp when it is added to the instance
2. Allow creation of built-in content filters with different type name
3. Set different content filter signatures for each requester
4. Allow ResourceLimitsQos serialization method compatibility
5. Solve Discovery Server race conditions
6. Properly delete secure endpoints if registration fails
7. Add condition for `string_builder` nullptr on XMLDynamicParser.cpp in case bound is 0
8. Create RPC example participants with default profile
9. Avoid Data Race in Reader Locator
10. Add multicast locator to correct locator list in ReaderProxydata
11. Fix ParticipantGenericMessage serialization
12. Fix build on QNX
13. Fix DS initialization on “any” if no interfaces are available
14. Avoid sending duplicated ACKs in DataSharing
15. Allow empty partition list to match against `"*"`
16. Release change while authentication fails
17. Change non_blocking_send threshold to `sendBufferSize`
18. Remote writers considered volatile by default
19. Solve TCP disconnect while incomplete read deadlock
20. Handle maximum deadline misses case
21. Fix duplicated transport when using specific XML with CLI
22. Add localhost capability only to UDP & TCP transports
23. Fix doxygen documentation for `iHandle2GUID`
24. Avoid setting thread affinity of 0 in mac
25. Add missing field to `xsd`
26. Fix topic argument in Discovery Server example
27. Add missing IDL to code generator script
28. Do not export discovery server as cmake target

This release includes the following **CI and release management updates**:

1. Move `SimpleCommunication` and `system` tests in Windows nightly
2. Add compilation with latest `GCC` to `fastdds_alternative_builds` Ubuntu CI
3. Remove `2.10.x` from PR template
4. Remove `3.1.x` from PR template and nightly for `3.1.x` and `2.10.x`
5. Move `2.10.x` to EOL versions
6. Update `3.2.x` to be a supported branch
7. Split windows workflow
8. Correct Python fallback branch
9. Add `uncrustify` to Github CI
10. Install Fix OpenSSL version in windows example testing
11. Add VS143 `toolset` to Windows CI
12. Add `--quiet` to `git submodule` invocations
13. Fix doxygen to avoid errors in python bindings `docstrings`
14. Update fallback branches to use environment variables
15. Update fastcdr submodule to v2.3.3
16. Split Mac CI build and testing phases
17. Remove RPC example
18. Regenerate types with Fast DDS Gen 4.0.5
19. Use appropriate Discovery-Server repository branch

#### IMPORTANT
When upgrading to version 3.2.3 it is **highly recommended** to regenerate generated source from IDL files
using at least [Fast DDS-Gen v4.0.5](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.0.5).
But it is advisable to regenerate them using the latest patch version of `Fast DDS-Gen v4.0.x`.

<a id="id8"></a>

#### [Version 3.2.2](https://fast-dds.docs.eprosima.com/en/v3.2.2/index.html)

This release includes the following **fixes**:

1. Fix `related_sample_identity` handling in RPC for ROS 2 request messages
2. Fix compilation in Windows using MinGW compiler
3. Fix a crash error in `XMLDynamicParser.cpp` due to `wstring_builder` being created as `nullptr`

This release includes the following **improvements**:

1. Add support for sequence types in `IDLParser`
2. Improve efficiency of Discovery Server routines
3. Always copy `type_information` in `ReaderProxyData` and `WriterProxyData` copy constructors
4. Add Fast DDS v2.6.10 to supported releases

This release includes the following **ci management updates**:

1. Force using asio from thirdparty in MacOS CI
2. Update 2.6.x weekly Ubuntu image to Ubuntu 22.04

#### IMPORTANT
When upgrading to version 3.2.2 it is **highly recommended** to regenerate generated source from IDL files
using at least [Fast DDS-Gen v4.0.4](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.0.4).
But it is advisable to regenerate them using the latest patch version of `Fast DDS-Gen v4.0.x`.

<a id="id9"></a>

#### [Version 3.2.1](https://fast-dds.docs.eprosima.com/en/v3.2.1/index.html)

This patch release includes the following **fixes**:

1. Reduce trace category when failing to unregister type
2. Avoid sending statistics message with big messages and no fragmentation

This release includes the following **improvements**:

1. Setup `ROS 2 Easy Mode` at participant creation
2. Update CLI commands for Easy Mode
3. Get correct Fast CDR related branch in CI
4. Update excluded IDL files from `Gen` CI

This release includes the following **test fixes**:

1. Fix `MacOS` nightly flaky tests
2. IDL parsing tests included paths
3. Fix Windows example tests in `Debug` and mitigate rtps example flakiness

#### IMPORTANT
When upgrading to version 3.2.1 it is **highly recommended** to regenerate generated source from IDL files
using at least [Fast DDS-Gen v4.0.4](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.0.4).
But it is advisable to regenerate them using the latest patch version of `Fast DDS-Gen v4.0.x`.

<a id="id11"></a>

#### [Version 3.2.0](https://fast-dds.docs.eprosima.com/en/v3.2.0/index.html)

This minor release includes the following **features**:

1. Implementation of `get_matched_publication/subscription()` `DataReader/Writer` API
2. Extended incompatible QoS for monitor service
3. Transform locators using new `machine_id PID`
4. New property to select [preferred key agreement algorithm.](../../fastdds/property_policies/security.md#property-policies-security)
5. Parse IDL file on [DynamicTypeBuilderFactory::create_type_w_uri](../../fastdds/xtypes/idl_parsing.md#dynamic-types-idl-parsing)
6. Fast DDS [ROS2 Easy Mode](https://docs.vulcanexus.org/en/latest/rst/enhancements/easy_mode/easy_mode.html)
7. Add [RPC](../../fastdds/rpc_dds/rpc_dds_intro.md#rpc-dds-intro) related interfaces
8. Extend `QoS` in discovery data

This release includes the following **improvements**:

**Transport:**

1. Check if SHM transport is disabled in `LARGE_DATA` modes
2. Allow domain names (DNS) when parsing TCP locators XML elements
3. Improve `Blackbox` TCP tests suite
4. Decouple transport receivers creation using unique network flows
5. Improve `max_allocations` calculation on SHM transport
6. Improve SHM configuration on large data transports

**Discovery:**

1. Improve `PDPClients` initialization
2. Make the machine id the physical_data host info
3. Filter interested readers on `PDP` writer
4. Make `*ProxyData` inherit from `*BuiltinTopicData` and refactor it
5. [Flow controller configuration for built-in endpoints](../../fastdds/use_cases/large_data/large_data.md#flow-controllers)

**Security:**

1. Improve `OpenSSL` lifecycle handling
2. Use correct algorithm strings on `PermissionsToken` and `IdentityToken`
3. Log any errors before `cancel_init()`
4. Add test for security initialization error

**Tests:**

1. Add test for a union using an alias with scope
2. Tests for regression test: fix sequence of alias of primitives inside modules

**Tools and examples:**

1. Add `-v` or `--version` to [CLI command tool](../../fastddscli/cli/cli.md#ffastddscli-cli)
2. Enable [Datasharing segments clean from CLI](../../fastddscli/cli/cli.md#cli-shm)
3. Update examples `CMakeList` to force default `CMAKE_BUILD_TYPE`
4. Update `Benchmark` example
5. Remove `SHM` option from discovery server example

**Documentation:**

1. Roadmap update
2. Update latest notes from `2.10.x` to `2.10.6`
3. Update RELEASE_SUPPORT after `v2.14.4` release
4. Set `3.0.x` as `EOL`
5. Update RELEASE_SUPPORT after `3.0.2` release
6. Correct examples on README
7. Warn about `XTypes` in `Fast DDS v3`

**Other:**

1. Rename `RTPSParticipantImpl.h` to `RTPSParticipantImpl.hpp`
2. Update `sqlite` from `3.36.0` to `3.47.2`
3. Add modules support to `idl_serialize`

**Release management:**

1. Add replace tag to `package.xml`
2. Prepare for Release `v3.2.0`

**Github CI management:**

1. Add Debug CI to master nightly job
2. Run examples in Windows CI
3. Update submodules when cloning Fast DDS on CI
4. Split Windows nightly tests using `name filter regex`
5. Add missing `psutil` python module in sanitizers CI

This release includes the following **fixes**:

1. Fix [CVE-2025-24807](https://nvd.nist.gov/vuln/detail/cve-2025-24807)
2. Remove unused code
3. Fix destruction data-race on participant removal in intra-process
4. `SecurityManager` fixes
5. Address `XMLDynamicParser` regression
6. Fix assertion deleting a `DataWriter` configured with persistent and flow controller
7. Fix comparison in `is_update_allowed()`
8. Fix build with `-Werror=template-id-cdtor`
9. Fix commercial support link
10. Fix `cmake` generator evaluation
11. Fix reach of maximum buffers in `asio::send_to`
12. Fix unique network flows with TCP transports
13. Fix `tsan` potential deadlock between `StatefulWriter` and `FlowController`
14. Fix Discovery CLI Tool in Windows
15. Arithmetic overflow in fragment size calculations
16. Fix `double-locking` issue in `DataSharingListener`
17. Fix `EDP` reliability timings
18. Handle socket buffer size setting when `system's` maximum exceeded
19. Fix log category name macro collision in `MacOS`
20. Reliable volatile change dropped when all history `acked`
21. Increase `max_blocking_time` for services in `Easy Mode`
22. Unacknowledged sample removed in `KEEP_ALL` mode
23. Fix System Log error infinite loop when setting thread affinity
24. Support compiler `MSYS2-MinGW`
25. Fix python warning in `shm` CLI tool
26. Fix error handling logic in `try_setting_buffer_size`
27. Remove repeated header include and fix log
28. Flush logs in `CliDiscoveryManager` destructor
29. Fix cleanup race condition with exclusive and shared lock files
30. Fix assertion on `OutputTrafficManager`
31. Fix `several_writers_on_unack_sample_removed` flaky test

#### IMPORTANT
When upgrading to version 3.2.0 it is **highly recommended** to regenerate generated source from IDL files
using at least [Fast DDS-Gen v4.0.4](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v4.0.4).
But it is advisable to regenerate them using the latest patch version of `Fast DDS-Gen v4.0.x`.

<a id="version-2-14-maintenance"></a>

### Version 2.14 (maintenance)

<a id="id13"></a>

#### [Version 2.14.7 (maintenance)](https://fast-dds.docs.eprosima.com/en/v2.14.7/index.html)

#### IMPORTANT
Fast DDS v2.14 is the last minor version of Fast DDS v2, the major release Fast DDS
v3 is already out! Check out all the new features and improvements in <[https://fast-dds.docs.eprosima.com/en/latest/](https://fast-dds.docs.eprosima.com/en/latest/)>

This release includes the following **security fixes**:

- [Fix CVE-2026-22591](https://www.cve.org/CVERecord?id=CVE-2026-22591)
- [Fix CVE-2026-45094](https://www.cve.org/CVERecord?id=CVE-2026-45094)
- [Fix CVE-2026-45095](https://www.cve.org/CVERecord?id=CVE-2026-45095)
- [Fix CVE-2026-45096](https://www.cve.org/CVERecord?id=CVE-2026-45096)
- [Fix CVE-2025-63829](https://www.cve.org/CVERecord?id=CVE-2025-63829)
- [Fix CVE-2025-65865](https://www.cve.org/CVERecord?id=CVE-2025-65865)
- [Fix CVE-2026-45093](https://www.cve.org/CVERecord?id=CVE-2026-45093)
- [Fix CVE-2026-45092](https://www.cve.org/CVERecord?id=CVE-2026-45092)
- [Fix CVE-2025-67108](https://www.cve.org/CVERecord?id=CVE-2025-67108)
- [Fix CVE-2026-49861](https://www.cve.org/CVERecord?id=CVE-2026-49861)
- [Fix CVE-2026-53589](https://www.cve.org/CVERecord?id=CVE-2026-53589)
- [Fix CVE-2026-45098](https://www.cve.org/CVERecord?id=CVE-2026-45098)
- [Fix CVE-2026-49862](https://www.cve.org/CVERecord?id=CVE-2026-49862)
- [Fix CVE-2026-55063](https://www.cve.org/CVERecord?id=CVE-2026-55063)
- [Fix CVE-2026-49863](https://www.cve.org/CVERecord?id=CVE-2026-49863)
- [Fix CVE-2026-53588](https://www.cve.org/CVERecord?id=CVE-2026-53588)
- [Fix CVE-2026-53590](https://www.cve.org/CVERecord?id=CVE-2026-53590)
- [Fix GHSA-8g7x-c9fg-g6mh](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-8g7x-c9fg-g6mh)

This release includes the following **improvements**:

1. Regenerate types with Fast DDS Gen 3.3.3
2. Update Fast CDR submodule

This release includes the following **fixes**:

1. Fix `RTPSParticipantAttributes` internal data races
2. Catch exception when constructing a payload pool Node
3. Fix dereference error of ‘shared_secret_handle’
4. Assert liveliness with periodic heartbeats in secure participants
5. Make `on_xxx_matched` thread-safe
6. Fix data races on `DataWriterImpl` `QoS` access
7. Remove stale TCP channels
8. Fix unused variable on GCC 16
9. Fix datasharing infinite loop
10. Fix shared-memory segment crash on port open

This release includes the following **CI improvements**:

1. Improve windows tests
2. Update security tests to use governance-based encryption configuration
3. Add `UBSan` workflow and solve its errors
4. Make `skip-ci` label report required statuses as pass
5. Fix Docs after HTML zip refactor

<a id="id31"></a>

#### [Version 2.14.6](https://fast-dds.docs.eprosima.com/en/v2.14.6/index.html)

#### IMPORTANT
Fast DDS v2.14 is the last minor version of Fast DDS v2, the major release Fast DDS
v3 is already out! Check out all the new features and improvements in <[https://fast-dds.docs.eprosima.com/en/latest/](https://fast-dds.docs.eprosima.com/en/latest/)>

This patch release includes the following security fixes:

- [Fix CVE-2025-62599](https://www.cve.org/CVERecord?id=CVE-2025-62599)
- [Fix CVE-2025-62600](https://www.cve.org/CVERecord?id=CVE-2025-62600)
- [Fix CVE-2025-62601](https://www.cve.org/CVERecord?id=CVE-2025-62601)
- [Fix CVE-2025-62602](https://www.cve.org/CVERecord?id=CVE-2025-62602)
- [Fix CVE-2025-62603](https://www.cve.org/CVERecord?id=CVE-2025-62603)
- [Fix CVE-2025-62799](https://www.cve.org/CVERecord?id=CVE-2025-62799)
- [Fix CVE-2025-64098](https://www.cve.org/CVERecord?id=CVE-2025-64098)
- [Fix CVE-2025-64438](https://www.cve.org/CVERecord?id=CVE-2025-64438)
- [Fix CVE-2025-65016](https://www.cve.org/CVERecord?id=CVE-2025-65016)
- [Fix CVE-2026-22590](https://www.cve.org/CVERecord?id=CVE-2026-22590)

This patch release includes the following improvements:

- Allow to send empty payloads in dispose/unregister ops
- Reset irrelevant sequence numbers interval in proxy readers
- Improve Open SSL includes
- Verify Safe DDS signature
- Replace logWarning with EPROSIMA_LOG_WARNING
- Replace `logWarning` with `EPROSIMA_LOG_WARNING`
- Improvements in message receiver
- Improve some tests
- Protect PDPStatelessWriter attributes
- Avoid casting GUID_t into InstanceHandle_t
- Avoid casting `GUID_t` into `InstanceHandle_t`
- Protect DataReaderInstance `alive_writers` attribute
- Fix wrong namespace when indicating CDR version
- Allow empty partition list to match against “\*”
- Apply `Uncrustify` to 2.14.x
- Avoid sending duplicated ACKs in DataSharing
- Send heartbeat when wait_for_all_acked is called
- Send heartbeat when `wait_for_all_acked` is called
- Solve TCP disconnect while incomplete read deadlock
- Remote writers considered volatile by default
- Change non_blocking_send threshold to `sendBufferSize`
- Regenerated types using fastddsgen 3.3.1
- Use binary search in `find_change_nts`
- Support aliases in Content Filtered Topic
- Update fallback branches to use environment variables
- Add missing field to `xsd`
- Avoid setting thread affinity of 0 in mac
- Revert “Return sample notifying changes on instance state”
- Regenerate types with Fast DDS Gen 3.3.2
- Update fastcdr submodule to point to v2.2.7

This patch release includes the following fixes:

- Release change while authentication fails
- Fix DataReader history enforcement to respect max_samples_per_instance
- Fix wrong update of default locators
- Fix build in GCC 15
- Fix GAP messages are not sent when there is no Reader requesting the DATA

This patch release includes the following CI improvements:

- Upgrade `Linters` CI runner image to `ubuntu-24.04`
- Free disk space in Ubuntu runners
- Single line for `ctest_args` in Mac CI
- Upgrade to macOS Sequoia
- Reduce disk usage in TSAN job
- Allow CI to run tests with manual dispatch
- Split Mac CI build and testing phases

#### NOTE
When upgrading to version 2.14.6 it is **advisable** to regenerate generated source from IDL files
using [Fast DDS-Gen v3.3.2](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.3.2).

<a id="id42"></a>

#### [Version 2.14.5](https://fast-dds.docs.eprosima.com/en/v2.14.4/index.html)

#### IMPORTANT
Fast DDS v2.14 is the last minor version of Fast DDS v2, the major release Fast DDS
v3 is already out! Check out all the new features and improvements in <[https://fast-dds.docs.eprosima.com/en/latest/](https://fast-dds.docs.eprosima.com/en/latest/)>

This patch release includes the following **features**:

1. New property to select [preferred key agreement algorithm](../../fastdds/security/auth_plugin/auth_plugin.md#auth-pki-dh).

This patch release includes the following **security fixes**:

1. [CVE-2025-24807](https://www.cve.org/CVERecord?id=CVE-2025-24807)

This patch release includes the following **improvements**:

1. Decouple transport receivers creation using unique network flows
2. Improve `max_allocations` calculation on SHM transport
3. Improve `PDPClients` initialization
4. Filter interested readers on `PDP` writer
5. Refactor builtin writers and readers creation
6. Update reception timestamp when it is added to the instance
7. Unacknowledged sample removed in `KEEP_ALL` mode
8. Add localhost capability only to UDP and TCP transports
9. Avoid data race in `ReaderLocator`
10. Improve `DS` routines
11. Handle socket buffer size setting when system’s maximum exceeded
12. Improve `Blackbox` TCP tests suite
13. Avoid calling `setIPv4` in `TCPTransportInterface`
14. Avoid sending statistics message with big messages and no fragmentation
15. Fix for `Configurability` example
16. Increase the announcement period in liveliness test
17. Allow creation of built-in content filters with different type name
18. Return sample notifying changes on instance state
19. Update `sqlite` from `3.36.0` to `3.47.2`
20. Log any errors before `cancel_init()`
21. Add `--quiet` to git `submodule` invocations
22. Bump to `asio 1.34.2`
23. Allow DNS when parsing TCP locators XML elements

This patch release includes the following **fixes**:

1. Add pointer sanity check before accessing `ParticipantProxyData`
2. Fix `CMake` generator evaluation
3. Fix unique network flows with TCP transports
4. Fix TCP discovery server locators translation
5. Arithmetic overflow in fragment size calculations
6. Fix `double-locking` issue in `DataSharingListener`
7. Fix comparison in `is_update_allowed`
8. Use correct algorithm strings on `PermissionsToken` and `IdentityToken`
9. Fix `tsan potential deadlock` between `StatefulWriter` and `FlowController`
10. Reliable volatile change dropped when all history is `acked`
11. Fix log category name macro collision in MacOS
12. Fix `null-dereference` in `parseXMLMemberDynamicType`
13. Fix infinite loop when setting thread affinity
14. Fix build with `-Werror=template-id-cdtor`
15. Fix error handling logic in `try_setting_buffer_size`
16. Remove repeated header include and Fix log
17. Fix `several_writers_on_unack_sample_removed` flaky test
18. Fix cleanup race condition with exclusive and shared lock files
19. Address some compilation warnings with `GCC` latest
20. Fix `ParticipantGenericMessage` serialization
21. Fix build on `QNX`
22. Fix assertion on `OutputTrafficManager`
23. Fix Data `100kb CDR` version
24. Fix socket buffer size handling
25. Fix\`\` DS\`\` initialization on `any` if no interfaces are available
26. Fix `EDP` reliability timings
27. Fix compatibility with `Fast CDR v1`
28. Regression test for fuzz `XMLParser null-dereference`

This patch release includes the following **CI improvements**:

1. Update `submodules` when cloning Fast DDS on CI
2. Force `Asio_Thirdparty` in MacOS CI
3. Add `uncrustify` to Github CI
4. Get correct Fast CDR related branch
5. Support compiler `MSYS2-MinGW`
6. Use `windows-2022` in CI

#### NOTE
When upgrading to version 2.14.5 it is **advisable** to regenerate generated source from IDL files
using [Fast DDS-Gen v3.3.1](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.3.1).

<a id="id43"></a>

#### [Version 2.14.4](https://fast-dds.docs.eprosima.com/en/v2.14.4/index.html)

#### IMPORTANT
Fast DDS v2.14 is the last minor version of Fast DDS v2, the major release Fast DDS
v3 is already out! Check out all the new features and improvements in <[https://fast-dds.docs.eprosima.com/en/latest/](https://fast-dds.docs.eprosima.com/en/latest/)>

# Fast DDS v2.14.4

This release includes the following **features** in an ABI compatible manner:

1. Add unsigned specification to literals.
2. Allow running CI on external contributions.
3. Add references to new nightly jobs per supported branch in README.

This release includes the following **improvements**:

1. Add Ubuntu weekly CI.
2. Fix Windows CI and add vanilla build step in Ubuntu CI.
3. Use eProsima-CI action to install Qt.
4. Modify help command of XML CLI tool.
5. Update commercial support section in README.

This release includes the following **fixes**:

1. Mitigate LogTests.flush_n flakiness.
2. Mitigate buffer recover test flakiness on mac.
3. Fix Python version in sanitizers CI.
4. Fix OSS fuzz build.
5. Fix warning in OSS fuzz build.
6. Change monitor service writer entity ID.
7. Solve SecurityManager memory issue.
8. Fix issue with exclusive ownership and unordered samples.
9. Fix data race in TypeObjectFactory::get_instance.
10. Fix: Secure simple participants with initial peers over TCP match.
11. Remove double // in some installation paths.
12. Discard changes with big key-only payload and no key hash.
13. Fix DataReaderHistory regression.
14. Fix Secure Discovery Server client disposals GUID and handshake_handle assertion.
15. Fix issues in Dynamic Network Interfaces.
16. Check if SHM transport is disabled in LARGE_DATA modes.
17. Release participant_stateless secure builtin writer history change when authentication has finished.
18. Improve OpenSSL lifecycle handling.
19. Fix destruction data race on participant removal in intra-process.
20. Be less strict with parameter lengths.

#### NOTE
When upgrading to version 2.14.4 it is **advisable** to regenerate generated source from IDL files
using [Fast DDS-Gen v3.3.1](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.3.1).

<a id="id45"></a>

#### [Version 2.14.3](https://fast-dds.docs.eprosima.com/en/v2.14.3/index.html)

#### IMPORTANT
Fast DDS v2.14 is the last minor version of Fast DDS v2, take a look at Fast DDS v3!

This release includes the following **features** in an ABI compatible manner:

1. Setting `vendor_id` on received `CacheChange_t`

This release includes the following **improvements**:

1. Repository & CI improvements.
2. Create InitialConnection for [TCP Transport](../../fastdds/transport/tcp/tcp.md#transport-tcp-tcp) initial peers

This release includes the following **fixes**:

1. Fix topic interference on `liveliness_changed` status
2. Remove doxygen warnings
3. Adjust doxygen so Fast DDS Python pydoc makes RTD build
4. Fix compilation in Ubuntu 24.04 platforms
5. Fix Latency test destruction

#### NOTE
When upgrading to version 2.14.3 it is **advisable** to regenerate generated source from IDL files
using [Fast DDS-Gen v3.3.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.3.0).

<a id="id46"></a>

#### [Version 2.14.2](https://fast-dds.docs.eprosima.com/en/v2.14.2/index.html)

#### IMPORTANT
Fast DDS v2.14 is the last minor version of Fast DDS v2, take a look at Fast DDS v3!

This release includes the following **features** in an ABI compatible manner:

1. Set [DataSharing Qos policy](../../fastdds/dds_layer/core/policy/eprosimaExtensions.md#datasharingqospolicy) in transmitted `WriterProxyData` and `ReaderProxyData`.
2. New [max_message_size](../../fastdds/use_cases/tcp/tcp_large_data_with_options.md#use-case-large-data-options) property to limit the output datagrams size.
3. Add XML configuration for [Flow Controllers](../../fastdds/use_cases/large_data/large_data.md#flow-controllers).

This release includes the following **improvements**:

1. Repository & CI improvements.
2. Do not require `PYTHON_VERSION` to be defined. in .bat files.
3. Use `%*` instead of loop in `.bat` scripts.
4. Use absolute paths when loading XML files.
5. Bump Fast CDR submodule to version 2.2.2.

This release includes the following **fixes**:

1. Handle errors when setting socket buffer sizes.
2. Do not require Fast CDR v2 in examples.
3. Fix [Shared Memory Transport](../../fastdds/transport/shared_memory/shared_memory.md#transport-sharedmemory-sharedmemory) buffer recovery MacOS flaky test.
4. Automatically unmatch remote participants on participant deletion.
5. Bugfix: correct [liveliness](../../fastdds/dds_layer/core/policy/standardQosPolicies.md#livelinessqospolicy) state in a multiple reader - one writer scenario.
6. Only apply [content filter](../../fastdds/dds_layer/topic/contentFilteredTopic/contentFilteredTopic.md#dds-layer-topic-contentfilteredtopic) to ALIVE changes.
7. Properly delete builtin statistics writers upon `delete_contained_entities()`.
8. Fix doxygen warning about undocumented `@param` in deleted functions.
9. Correctly initialize `MatchingFailureMask` constants to be used with the `std::bitset` API.
10. Fix [Discovery Server](../../fastdds/discovery/discovery_server.md#discovery-server) not connecting due to ports logic.

#### NOTE
When upgrading to version 2.14.2 it is **advisable** to regenerate generated source from IDL files
using [Fast DDS-Gen v3.3.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.3.0).

<a id="id48"></a>

#### [Version 2.14.1](https://fast-dds.docs.eprosima.com/en/v2.14.1/index.html)

#### IMPORTANT
Fast DDS v2.14 is the last minor version of Fast DDS v2, take a look at Fast DDS v3!

This release includes the following **features** in an ABI compatible manner:

1. Implement copy_from_topic_qos method (see [Creating a DataWriter](../../fastdds/dds_layer/publisher/dataWriter/createDataWriter.md#dds-layer-publisher-datawriter-creation) and see
   [Creating a DataReader](../../fastdds/dds_layer/subscriber/dataReader/createDataReader.md#dds-layer-subscriber-datareader-creation)).

This release includes the following **improvements**:

1. Set 2.12.x as EOL
2. Use a plain switch for NetmaskFilterKind operator<<
3. Set real TCP non_blocking_send limitation
4. Increase ack waiting time in reliable_on_unack_sample_removed
5. Enforce SHM ports open mode exclusions
6. Force unlimited ResourceLimits if lower or equal to zero (see [ResourceLimitsQosPolicy](../../fastdds/dds_layer/core/policy/standardQosPolicies.md#resourcelimitsqospolicy))
7. Allow processing of AckNack submessages with `count == 0`
8. Refactor `IStatusQueryable` and make monitor service interfaces private
9. Internal refactor on port handling
10. Improve `ThreadSettingsQoS` logging
11. Run `is_plain` method with the corresponding data representation
12. Address compilation issue for GLIBC version
13. Repository and test improvements:
    > 1. Run Github Ubuntu CI on PRs
    > 2. Only run PRs CI when a review is requested
    > 3. Improve filtering of DNS tests
    > 4. Make `sample_lost_be_dw_be_dr_fragments` test less flaky
    > 5. Build Fast DDS Python bindings in Fast DDS Docs Github CI job
    > 6. Add check for XML API to PR template
    > 7. Refactor Github CI sanitizer related jobs
    > 8. Increase sleep to miss the deadline in macOS flaky tests
    > 9. Fix nightly sanitizer CI workflow name
    > 10. Improve xml loading method in fuzz_XMLProfiles
    > 11. Build ShapesDemo on Ubuntu Github CI
    > 12. Correctly set branches for `reusable-sanitizers-ci`

This release includes the following **fixes**:

1. Address XMLProfiles fuzzer regressions
2. Fix flaky Log tests
3. Fix hidden overloaded virtual methods
4. Effectively assert `automatic/manual_by_participant` liveliness
5. Fix `on_sample_lost` notification on best-effort readers for fragmented samples
6. Monitor service properly managing instances
7. Fix CVE-2024-30258
8. Fix Discovery Server over TCP using `LocatorSelectorEntry`
9. Removed warning
10. Make `get_first_untaken_info()` coherent with `read()/take()`
11. Fix serialization of Data(w)/Data(r) in MonitorService
12. Bugfix in `DDSBlackboxTestsDataReader` test
13. Fix leak in `SecurityManager::participant_volatile_message_secure_writer_`
14. Fix some leaks in XML DynamicTypes Parser
15. Fix support for `@key` annotation in Dynamic types
16. Fix SecurityTest unit tests memory error
17. Fix Python Installation version in Github CI. Address failing system tests environment issues

#### NOTE
When upgrading to version 2.14.0 it is **advisable** to regenerate generated source from IDL files
using [Fast DDS-Gen v3.3.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.3.0).

<a id="id50"></a>

#### [Version 2.14.0](https://fast-dds.docs.eprosima.com/en/v2.14.0/index.html)

#### IMPORTANT
Fast DDS v2.14 is the last minor version of Fast DDS v2, take a look at Fast DDS v3!

This release includes the following **new features**:

1. Add [netmask filter](../../fastdds/transport/interfaces.md#ifaces-config) transport configuration
2. Add [interface allowlist and blocklist](../../fastdds/transport/interfaces.md#ifaces-config)
3. Expose [Security Authentication plugin](../../fastdds/property_policies/security.md#property-policies-security) Handshake Properties
4. Extend `LARGE_DATA` [configuration options](../../fastdds/use_cases/tcp/tcp_large_data_with_options.md#use-case-large-data-options)
5. Allow [single listening port on TCP](../../fastdds/transport/tcp/tcp.md#transport-tcp-transportdescriptor)
6. OpenOutputChannels / CloseOutputChannels that receive a LocatorSelectorEntry
7. Support GTest v1.14.0
8. Update Fast CDR submodule to v2.2.0
9. Update roadmap and release support

This release includes the following **improvements**:

1. Ignore custom PIDs defined in Fast DDS when they are received from other vendors
2. Add catch of out-of-range exception for thread settings port
3. Explicitly pass vendor ID to `readFromCdrMessage`
4. TCP transport improvements:
   > 1. `TCPSendResources` cleanup
   > 2. TCP `non_blocking_send` moved to [TCPTransportDescriptor](../../fastdds/transport/tcp/tcp.md#transport-tcp-transportdescriptor)
5. Various repository improvements and housekeeping:
   > 1. Migrate communication tests to the DDS API
   > 2. Migrate TCPReqRepHelloWorldReplier/Requester to the DDS API
   > 3. Update APIs in video performance tests to the DDS API
   > 4. Remove `FASTDDS_TODO_BEFORE` 2.14
   > 5. Remove use of deprecated FindPythonInterp
   > 6. Remove idl parser from .repos file
   > 7. Force usage of semicolon in `FASTDDS_TODO_BEFORE` macro
   > 8. Ubuntu example testing automation infrastructure
6. Multiple Github CI improvements:
   > 1. Add nightly Ubuntu Github CI
   > 2. Improve CI version management
   > 3. Avoid running GitHub CI if PR has conflicts
   > 4. Migrate apt package installation to eProsima-CI action
   > 5. Include missing nightly for 3.0.x branch
   > 6. Adapt nightly jobs for all supported versions
   > 7. Pin CMake version and vm.mmap_rnd_bits in sanitizer workflows
   > 8. Select reusable workflow version depending on target branch in nightly jobs

This release includes the following **fixes**:

1. Fix warnings when compiling for Windows x86
2. TCP first message loss (see [TCPTransportDescriptor](../../fastdds/transport/tcp/tcp.md#transport-tcp-transportdescriptor))
3. Fix warnings on tests on Windows 32bits
4. Protect asio exception hotfix
5. Fix CVE-2024-28231
6. Github CI fixes:
   > 1. Fix CI documentation workflow label triggering
   > 2. Fix nightly jobs
   > 3. Fix input branch on reusable windows CI
   > 4. Use correct version of GTest on DS ASan tests

#### NOTE
When upgrading to version 2.14.0 it is **advisable** to regenerate generated source from IDL files
using [Fast DDS-Gen v3.3.0](https://github.com/eProsima/Fast-DDS-Gen/releases/tag/v3.3.0).

<a id="version-2-6-maintenance"></a>

### Version 2.6 (maintenance)

<a id="id52"></a>

#### [Version 2.6.12 (maintenance)](https://fast-dds.docs.eprosima.com/en/v2.6.12/index.html)

This release includes the following **security fixes**:

- [Fix CVE-2026-22590](https://www.cve.org/CVERecord?id=CVE-2026-22590)
- [Fix CVE-2026-22591](https://www.cve.org/CVERecord?id=CVE-2026-22591)
- [Fix CVE-2026-45094](https://www.cve.org/CVERecord?id=CVE-2026-45094)
- [Fix CVE-2026-45095](https://www.cve.org/CVERecord?id=CVE-2026-45095)
- [Fix CVE-2026-45096](https://www.cve.org/CVERecord?id=CVE-2026-45096)
- [Fix CVE-2025-63829](https://www.cve.org/CVERecord?id=CVE-2025-63829)
- [Fix CVE-2025-65865](https://www.cve.org/CVERecord?id=CVE-2025-65865)
- [Fix CVE-2026-45093](https://www.cve.org/CVERecord?id=CVE-2026-45093)
- [Fix CVE-2026-45092](https://www.cve.org/CVERecord?id=CVE-2026-45092)
- [Fix CVE-2025-67108](https://www.cve.org/CVERecord?id=CVE-2025-67108)
- [Fix CVE-2026-49861](https://www.cve.org/CVERecord?id=CVE-2026-49861)
- [Fix CVE-2026-53589](https://www.cve.org/CVERecord?id=CVE-2026-53589)
- [Fix CVE-2026-45098](https://www.cve.org/CVERecord?id=CVE-2026-45098)
- [Fix CVE-2026-49862](https://www.cve.org/CVERecord?id=CVE-2026-49862)
- [Fix CVE-2026-55063](https://www.cve.org/CVERecord?id=CVE-2026-55063)
- [Fix CVE-2026-49863](https://www.cve.org/CVERecord?id=CVE-2026-49863)
- [Fix CVE-2026-53588](https://www.cve.org/CVERecord?id=CVE-2026-53588)
- [Fix CVE-2026-53590](https://www.cve.org/CVERecord?id=CVE-2026-53590)

This release includes the following **fixes**:

1. Several fixes in `TopicPayloadPool`
2. Fix lock order inversion in `StatefulWriter`
3. Fix `StatefulReader` iterator invalidation during listener callback
4. Check endianness of `TCPControlMsgHeader` & `TCPHeader`

This release includes the following **CI improvements**:

1. Upgrade Linters CI runner image to `ubuntu-24.04`

<a id="id71"></a>

#### [Version 2.6.11 (maintenance)](https://fast-dds.docs.eprosima.com/en/v2.6.11/index.html)

#### IMPORTANT
According to our
[release support guidelines](https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md)
the v2.6 minor will only receive patches for critical issues and security fixes.

This release includes the following **critical fixes**:

1. Fix [CVE-2025-62599](https://www.cve.org/CVERecord?id=CVE-2025-62599)
2. Fix [CVE-2025-62600](https://www.cve.org/CVERecord?id=CVE-2025-62600)
3. Fix [CVE-2025-62601](https://www.cve.org/CVERecord?id=CVE-2025-62601)
4. Fix [CVE-2025-62602](https://www.cve.org/CVERecord?id=CVE-2025-62602)
5. Fix [CVE-2025-62603](https://www.cve.org/CVERecord?id=CVE-2025-62603)
6. Fix [CVE-2025-64098](https://www.cve.org/CVERecord?id=CVE-2025-64098)
7. Fix [CVE-2025-62799](https://www.cve.org/CVERecord?id=CVE-2025-62799)
8. Fix [CVE-2025-64438](https://www.cve.org/CVERecord?id=CVE-2025-64438)
9. Fix [CVE-2025-65016](https://www.cve.org/CVERecord?id=CVE-2025-65016)

This release includes the following **improvements**:

1. Verify Safe DDS signature
2. Improvements in message receiver
3. Regenerate types with Fast DDS Gen 2.1.4
4. Upgrade to Fast CDR v1.0.29

This release includes the following **ci management updates**:

1. Add `uncrustify` to Github CI
2. Remove deprecated windows 2019 from CI in favor of 2022
3. Upgrade to macOS Sequoia

<a id="id72"></a>

#### [Version 2.6.10 (maintenance)](https://fast-dds.docs.eprosima.com/en/v2.6.10/index.html)

#### IMPORTANT
According to our
[release support guidelines](https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md)
the v2.6 minor will only receive patches for critical issues and security fixes.

This release includes the following **critical fixes**:

1. Fix [CVE-2025-24807](https://www.cve.org/CVERecord?id=CVE-2025-24807)
2. Discard changes with big `key-only` payload and no key hash
3. Update types regeneration script `homing` path
4. Fix topic interference on `liveliness_changed` status

This release includes the following **improvements**:

1. Add `unsigned` specification to literals
2. Update commercial support section in README
3. Update `sqlite` from `3.36.0` to `3.47.2`

This release includes the following **ci management updates**:

1. Exclude flaky tests when compiling without security
2. Fix python version in sanitizers CI
3. Fix Windows CI and add vanilla build step in Ubuntu CI
4. Update submodules when cloning Fast DDS on CI
5. Add Ubuntu weekly CI
6. Allow running CI on external contributions
7. Use `eProsima-CI` action to install `Qt`
8. Add references to new nightly jobs per supported branch in README

<a id="id73"></a>

#### [Version 2.6.9](https://fast-dds.docs.eprosima.com/en/v2.6.9/index.html)

#### IMPORTANT
According to our
[release support guidelines](https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md)
Fast DDS v2.6.9 will be the last patch version receiving backported features and bugfixes.
From now on, the v2.6 minor will only receive patches for critical issues and security fixes.

This release includes the following **features**:

1. Add XML configuration for FlowControllerDescriptor to 2.x
2. New `max_message_size` property to limit output datagrams size

This release includes the following **improvements**:

1. Update Fast CDR thirdparty submodule
2. Consider library behavior changes as ABI breaks in the PR template checklist
3. Allow processing of AckNack submessages with count == 0
4. Use `%*` instead of loop in `.bat` scripts.
5. Use absolute paths when loading XML files
6. TCPSendResources cleanup

Github CI management:

1. Fix Python Installation version in Github CI. Address failing system tests environment issues
2. Set fallback branch for `get_related_branch_from_repo` correctly
3. Fix sanitizers CI test summary report
4. Protect asio exception
5. Set Fallback branch to 2.6.x
6. Run selected VS tool on Windows CI
7. Add DNS entries to hosts files on Github workflows
8. Refactor Fast DDS Ubuntu CI to include several tests
9. Avoid `CCache` in workflows and nighties
10. Update README.md with GitHub actions Ubuntu CI nightly
11. Label flaky tests with `xfail`

This release includes the following **fixes**:

1. Fix leak in `SecurityManager::participant_volatile_message_secure_writer_`
2. Fix Discovery Server over TCP
3. Fix some leaks in XML DynamicTypes Parser
4. Correct liveliness state in a multiple reader - one writer scenario
5. Fix support for `@key` annotation in Dynamic types
6. Properly delete builtin statistics writers upon `delete_contained_entities()`
7. Correctly initialize `MatchingFailureMask` constants to be used with the `std::bitset` API
8. Set DataSharing in `Writer|ReaderProxyData`
9. Only apply content filter to ALIVE changes
10. Handle errors when setting socket buffer sizes
11. Automatically unmatch remote participants on participant deletion
12. Fix on_sample_lost notification on best-effort readers for fragmented samples
13. Handle errors when setting socket buffer sizes
14. Fix DS servers not connecting due to ports logic
15. Manual fix for documentation generation
16. Create `InitialConnection` for TCP initial peers

<a id="id74"></a>

#### [Version 2.6.8](https://fast-dds.docs.eprosima.com/en/v2.6.8/index.html)

This release includes the following **features**:

1. [Authentication Handshake Properties](../../fastdds/property_policies/security.md#property-policies-security) documentation.
2. TCP Client and Server Participant Decision Making.

This release includes the following **improvements**:

1. Make DataWriters always send the key hash on keyed topics.
2. Include variety of terminate process signals handler in discovery server.
3. Effectively assert `AUTOMATIC/MANUAL_BY_PARTICIPANT` liveliness.
4. Pick smallest available participant ID for new participants.
5. Check History QoS inconsistencies.
6. Add check for XML API to PR template.
7. `LARGE_DATA` Participants logic with same listening ports.

TCP transport improvements:

1. TCP unique client announced local port.
2. Remove unnecessary TCP warning and Fix some tests.
3. TCP `non-blocking` send.
4. Enabling multiple interfaces through whitelist in TCP servers.
5. Set real TCP `non-blocking-send` limitation.

Github CI management:

1. Refactor Github CI sanitizer related jobs.
2. Avoid running GitHub CI if PR has conflicts.
3. Add manual Ubuntu Github CI.
4. Improve CI version management.
5. Build `ShapesDemo` on Ubuntu Github CI.
6. Only run PRs CI when review is requested.
7. Add macOS and Ubuntu Github CI.
8. Build Fast DDS Python bindings in Fast DDS Docs Github CI job.
9. Pin CMake version and `vm.mmap_rnd_bits` in sanitizer workflows.

This release includes the following **fixes**:

1. Fix and refactor Windows Github CI.
2. Fix max clash with Windows CI.
3. Fix the shared memory cleaning script.
4. Fix doxygen docs warnings. Prepare for compiling with `Doxygen 1.10.0`.
5. Prevent index overflow and correctly assert the end iterator in DataSharing.
6. Add a keyed fragmented change to the reader data instance only when its completed.
7. Add missing virtual destructor for `StatisticsAncillary`.
8. Fix wrong log info messages on TCP.
9. Migrate apt package installation action to `eProsima-CI`.
10. Fix CI documentation workflow label triggering.
11. Upgrade dependency version to last patch version in `.repos` file.
12. Fix `CVE-2024-28231`
13. Fix data race on PDP.
14. Discard already processed samples on PDPListener.
15. Fix flaky Log tests.
16. Add missing `TypeLookup` listeners.
17. Fix hidden overloaded virtual methods.
18. Fix TCP reconnection after open logical port failure.
19. Fix `CVE-2024-30258 / CVE-2024-30259`
20. Make [`DataReader::get_first_untaken_info()`](../../fastdds/api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader22get_first_untaken_infoEP10SampleInfo) coherent with `read()/take()`.
21. Removed warning in `ParameterList`.
22. TCP avoid first message loss.

<a id="id75"></a>

#### [Version 2.6.7](https://fast-dds.docs.eprosima.com/en/v2.6.7/index.html)

This release includes the following **features**:

1. Support `Autofill port` ([automatically set a port](../../fastdds/transport/tcp/tcp.md#transport-tcp-transportdescriptor)) for TCP Transport.
2. Define a [super client](../../fastdds/env_vars/env_vars.md#env-vars-ros-super-client) by environment variable
3. Support [TCP Discovery server](../../fastdds/use_cases/tcp/tcp_with_discovery_server.md#use-case-tcp-discovery-server) CLI and environment variable
4. Define methods ([environment variable](../../fastdds/env_vars/env_vars.md#env-vars-builtin-transports),
   [rtps layer](../../fastdds/rtps_layer/rtps_layer.md#rtps-layer-builtin-transports), [xml](../../fastdds/xml_configuration/domainparticipant.md#rtps)) to
   [configure transport scenarios](../../fastdds/transport/tcp/tcp.md#transport-tcp-enabling)
5. [Secure discovery server](../../fastdds/discovery/discovery_server.md#ds-security)

This release includes the following **improvements**:

1. Log warning message upon receiver resource creation failure.
2. Add tests for reconnection with same GUID
3. Use foonathan memory manager for reducing allocations in `SharedMemManager.hpp`
4. Simplify code in `CDRMessage`.
5. Rerun failed tests with ctest option instead of colcon’s.
6. Several improvements on CI jobs.
7. Upgrade CMake minimum requirement to 3.16.3
8. Update PR checklist template. Backports and Description

This release includes the following **bugfixes**:

1. Fix `DomainParticipant::register_remote_type` return when negotiating type.
2. Fix `RemoteBuiltinEndpointHonoring` blackbox test.
3. Allow participant profiles with no rtps tag.
4. Fix bad-free when receiving malformed DATA submessage.
5. Fix clang warnings
6. Use STL implementation of `Timed/RecursiveTimedMutex` when `MSVC >= 19.36`.
7. Fix encapsulation format in WLP.
8. Fix the clang build for clang 14.
9. Notify data-sharing listener at the end of a successful matching in intraprocess.
10. Updatable disable_positive_acks period.
11. Fix Data Race when updating liveliness changed in WLP.
12. Fix TCP sender resources creation.
13. Fix flow controllers unit tests compilation when using Fast CDR from thirdparty.
14. Add XML parser bit_bound bounds check.
15. Use `FASTRTPS_NO_LIB` on unittest root folder.
16. Use `SO_EXCLUSIVEADDRUSE` for Win32 unicast listening sockets.
17. Fix mac address overflow on windows.
18. Fix `PubSubAsReliable` test.
19. Fix `FileWatchTest`.
20. Add missing thread include.
21. Fix missing mandatory attribute check in XML parser struct type.
22. Better handling of trigger events in docs CI.
23. Fix memory problem when ciphering payload
24. Select correct .repos file on push events
25. Update TLS unit test certificates
26. Fix bad-free when receiving malformed DATA_FRAG submessage
27. Fix data race on writer destruction while sending heartbeat
28. Fix DiscoveryServer list access deadlock
29. Fix c++11 support for fast discovery server tool
30. Fix CVE-2023-50257
31. Fix std::move warning
32. Fix Github Windows CI

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id76"></a>

#### [Version 2.6.6](https://fast-dds.docs.eprosima.com/en/v2.6.6/index.html)

This release includes the following **improvements**:

1. Improve validation on PID_PROPERTY_LIST deserialization.
2. Improved CPU usage of timed events thread.
3. Improved performance on intraprocess + data-sharing.
4. Explicitly register type object in ContentFilteredTopicExample.
5. Improve installer generation with documentation version fallback.
6. Improve content filter expression parameters checks and verbosity.

This release includes the following **bugfixes**:

1. Fixed long-standing reconnection issues on SHM transport.
2. Correctly resolve alias in DDSQLFilter.
3. Fixed partition copy in QoS.
4. Added length checks to prevent nullptr memory copy calls.
5. Fixed XMLParser null-dereference when parsing log configuration.
6. Fixed SHM in 32-bit architectures.
7. Added missing include.
8. Avoid double definition of FASTDDS_ENFORCE_LOG_INFO.
9. Fixed statistics data_count with data-sharing.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id77"></a>

#### [Version 2.6.5](https://fast-dds.docs.eprosima.com/en/v2.6.5/index.html)

This release includes the following **improvements**:

1. Improve behavior when `STRICT_REALTIME` [CMake option](../../installation/configuration/cmake_options.md#cmake-options) is not enabled.
2. Using functors for `for_matched_readers` parameter.
3. Improve auto GAPs in Data Sharing.

This release includes the following **bugfixes**:

1. Fix RTPS StatelessWriter ACK check.
2. Fix `total_read_` to be consistent with Reader’s History after
   [`DataReader::get_first_untaken_info()`](../../fastdds/api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader22get_first_untaken_infoEP10SampleInfo).
3. Add deprecation notice to `ThroughputControllerDescriptor`.
4. UBSan (Undefined Behavior Sanitizer) fixes.
5. Several dependencies fixes upgrading to Ubuntu 22.04.
6. Fix chain of trust issues with a single CA certificate.
7. Correctly handle builtin endpoints mask.
8. Take mutex when removing local reader in WLP.
9. Handle `SIGTERM` signal in Fast DDS CLI.
10. Fix data races in SecurityManager authentication process.
11. Avoid creating entities within callbacks in DynamicHelloWorldExample.
12. Remove Asio dependency from DeadlineQoSExample.
13. Validity check for first sequence number.
14. Include right header when building for iOS.
15. Fix build on MSVC 19.
16. Correctly assign multicast port to multicast initial peers.
17. Select correct listener for
    [`on_requested_deadline_missed()`](../../fastdds/api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener28on_requested_deadline_missedEP10DataReaderRK29RequestedDeadlineMissedStatus).
18. Forward compatibility with boost inter-process 1.74+.
19. Fix missing includes when building with GCC 13.
20. Honor
    `allow_unauthenticated_participants`
    flag.
21. Capture all Fast CDR exceptions.
22. Fix example to delete Topic after deleting the corresponding Endpoint.
23. Protect against uncaught exception in SHM segment creation.
24. Initial acknack backoff.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id78"></a>

#### [Version 2.6.4](https://fast-dds.docs.eprosima.com/en/v2.6.4/index.html)

This release includes the following **bugfixes**:

1. Fix communication with asymmetric [Ignore Participant flags](../../fastdds/discovery/general_disc_settings.md#discovery-ignore-flags).
2. Fix deadlock in Writer Liveliness Protocol when using intraprocess.
3. Fix notification lost.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id79"></a>

#### [Version 2.6.3](https://fast-dds.docs.eprosima.com/en/v2.6.3/index.html)

This release includes the following **improvements**:

1. Upgrade CMake minimum requirement to 3.13.
2. Improve `Guid_t` operator `<` performance.

This release includes the following **bugfixes**:

1. Add python3 dependency to package.xml.
2. Fix complex member printing for DynamicDataHelper.
3. Fix selection of output locators.
4. Fix null references on XML parser.
5. Fix data races when creating DataWriters.
6. Send GAPs correctly when using separate sending.
7. Install Statistics IDL file.
8. Fixes for building in older compilers.
9. Fix deadlock when removing DomainParticipant when using SECURITY.
10. Ensure `shared_mutex` implementation is consistent throughout supported platforms.
11. Other minor fixes and improvements.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id80"></a>

#### [Version 2.6.2](https://fast-dds.docs.eprosima.com/en/v2.6.2/index.html)

This release includes the following **improvements**:

1. Support for GCC 12.
2. Overload [`DataReader::get_unread_count()`](../../fastdds/api_reference/dds_pim/subscriber/datareader.md#_CPPv4NK8eprosima7fastdds3dds10DataReader16get_unread_countEv).
3. Improve read/take performance when using topic with a great number of keys.
4. Improve rediscovery on lossy environments.

This release includes the following **bugfixes**:

1. Fixed several deadlocks and data races.
2. Fixed validation on `ParameterPropertyList_t`.
3. Fixed wrong usage of `std::remove_if`.
4. Fixed acknowledgement in DataSharing.
5. Other minor fixes.

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id81"></a>

#### [Version 2.6.1](https://fast-dds.docs.eprosima.com/en/v2.6.1/index.html)

This release includes the following **improvements**:

1. Support for [writer side content filtering](../../fastdds/dds_layer/topic/contentFilteredTopic/writerFiltering.md#dds-layer-topic-contentfilteredtopic-writer-side)
2. Support hexadecimal values on SQL filter
3. Support for [`DataWriter::get_key_value()`](../../fastdds/api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter13get_key_valueEPvRK16InstanceHandle_t)
4. Support for [`DataReader::lookup_instance()`](../../fastdds/api_reference/dds_pim/subscriber/datareader.md#_CPPv4NK8eprosima7fastdds3dds10DataReader15lookup_instanceEPKv)
5. Support for SampleLostStatus on DataReader
6. Improved doxygen documentation

Some **bugfixes** are also included:

1. Fixed several lock order inversion issues
2. Fixed data race when closing UDP channels
3. Fixed empty partition validation checks
4. Fixed corner case with reliable writers and samples with a huge number of fragments
5. Other minor fixes and improvements

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.

<a id="id82"></a>

#### [Version 2.6.0](https://fast-dds.docs.eprosima.com/en/v2.6.0/index.html)

This minor release is API compatible with the previous minor release, but introduces **ABI breaks** on two of the three
public APIs:

* Methods and attributes have been added on several classes of the DDS-PIM high-level API, so indexes of symbols on
  dynamic libraries may have changed. Some API is also being deprecated.
* Methods and attributes have been added on several classes of the RTPS low-level API, so indexes of symbols on dynamic
  libraries may have changed.
* Old Fast-RTPS high-level API remains ABI compatible.

This minor release includes the following **features**:

1. [Allow modifying remote server locators at runtime](../../fastdds/discovery/discovery_server.md#ds-modify-server-list)
2. [Add statistics physical information to DATA[p] using properties](../../fastdds/property_policies/non_consolidated_qos.md#property-policies-physical-data)
3. [Content filter discovery information RTPS API](../../fastdds/dds_layer/core/policy/eprosimaExtensions.md#contentfilterlimits)
4. Endpoint discovery RTPS API
5. `on_sample_lost` RTPS API
6. Transport layer API extension
7. [XML support for Fast DDS CLI](../../fastddscli/cli/cli.md#cli-discovery-cli)
8. [New exchange format to reduce bandwidth in Static Discovery](../../fastdds/property_policies/non_consolidated_qos.md#property-policies-edp-exchange-format)

It also includes the following **improvements**:

1. Support lowercase keywords on SQL filter
2. Separate initialization and enabling of BuiltinProtocols
3. Add `disable_positive_acks` to Static Discovery XML
4. Several updates in the DDS-PIM API
5. Support for octet vectors on XML parser
6. Update README and roadmap
7. Update Fast-CDR submodule to v1.0.24
8. Add new CMake option `APPEND_PROJECT_NAME_TO_INCLUDEDIR`

Some **bugfixes** are also included:

1. Fix MatchedStatus `last_*_handle`
2. Fix recommended statistics DataReaderQos to enable backwards compatibility
3. Fixes for supporting Python bindings in Windows platforms
4. Fix publishing physical data on statistics topic
5. Other minor fixes and improvements

#### NOTE
If you are upgrading from a version older than 1.7.0, it is **required** to regenerate generated source from IDL
files using *fastddsgen*.
If you are upgrading from any older version, regenerating the code is *highly recommended*.
