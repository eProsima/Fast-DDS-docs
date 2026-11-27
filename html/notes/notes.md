<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="release-notes"></a>

<a id="version-3-8-0-1-pro-latest"></a>

# Version 3.8.0.1 **[Pro]** (latest)

This release includes the following **security fixes**:

- [Fix GHSA-f5r5-m785-f5wg](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-f5r5-m785-f5wg)
- [Fix GHSA-w92j-25c7-68qx](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-w92j-25c7-68qx)
- [Fix GHSA-33xc-m88x-3cp8](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-33xc-m88x-3cp8)
- [Fix GHSA-f5r6-qvhq-mx93](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-f5r6-qvhq-mx93)
- [Fix GHSA-wfcj-cv96-9wj9](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-wfcj-cv96-9wj9)
- [Fix GHSA-g355-8h7f-6w8v](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-g355-8h7f-6w8v)
- [Fix GHSA-f6cw-h682-xw3p](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-f6cw-h682-xw3p)
- [Fix GHSA-mg54-9fm8-xvf3](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-mg54-9fm8-xvf3)
- [Fix GHSA-j4xc-fjx8-wfg4](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-j4xc-fjx8-wfg4)
- [Fix GHSA-6pwr-vh77-2x8m](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-6pwr-vh77-2x8m)
- [Fix GHSA-58x7-8g6g-mpp8](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-58x7-8g6g-mpp8)
- [Fix GHSA-57rp-jm6r-xjw5](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-57rp-jm6r-xjw5)
- [Fix GHSA-f4hw-9j9g-j8xj](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-f4hw-9j9g-j8xj)
- [Fix GHSA-6m2p-wjv5-386v](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-6m2p-wjv5-386v)
- [Fix GHSA-5fgv-8wxq-3f69](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-5fgv-8wxq-3f69)
- [Fix GHSA-w75r-338m-pc5x](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-w75r-338m-pc5x)
- [Fix GHSA-4r2x-mjm7-r285](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-4r2x-mjm7-r285)
- [Fix GHSA-vx4x-38rc-27w8](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-vx4x-38rc-27w8)
- [Fix GHSA-6prg-7f44-f36j](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-6prg-7f44-f36j)
- [Fix GHSA-m6f5-9h75-494p](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-m6f5-9h75-494p)
- [Fix GHSA-p8f8-79c6-vgvg](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-p8f8-79c6-vgvg)
- [Fix GHSA-3m58-hxq8-985j](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-3m58-hxq8-985j)
- [Fix GHSA-pj69-64h2-55wp](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-pj69-64h2-55wp)
- [Fix GHSA-ppg2-324j-crqq](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-ppg2-324j-crqq)
- [Fix GHSA-2rqq-x4j6-c3jm](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-2rqq-x4j6-c3jm)
- [Fix GHSA-f5cw-77h9-cvjj](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-f5cw-77h9-cvjj)

This release includes the following **features**:

1. Support inheritance in IDL parser
2. Support Bitmask on IDL Parser
3. New Congestion Control plugin: Reno-like
4. Support bitset in IDL Parser
5. Extend IP mobility to support allowlist (UDP only)
6. Support `TCPv6` with link-local interfaces
7. Add implementation for inconsistent topic status
8. Register types from every IDL file in a directory, searched recursively
9. New `PDPRelay` mode
10. Type Assignability
11. Add proprietary license validator
12. Add XML support for `DataRepresentationQosPolicy`
13. POSIX UDPv4 transport, and option to leave out `asio`

This release includes the following **fixes**:

1. Solve potential deadlock in `GrainedFlowController`
2. Fix minor data races
3. Fix SEGV in latency test
4. Fix default history depth in examples
5. Fix for array `idl_serialization` errors
6. Fix leak in UDP when opening input channels
7. Avoid using `latency_data_in_` when using loans in `LatencyTestPublisher`
8. Fix unexpected indentation
9. Select `IPv6` multicast interface by index instead of scope id
10. Fix `IPMonitor` shutdown delay
11. Fix misleading `UBSan` error report
12. Fix `UBSan` `GrainedFlowController` data-race destruction error
13. Fix license validation public API
14. Fix RPC endpoint enable result
15. Store the grant only once it has been domain filtered
16. Fix uninitialized variable
17. Configuration example fix: initialize missing variable
18. Bring fixes to `TopicPayloadPoolRegistry`
19. Fix deadlock updating attributes while pairing
20. Fix Windows build and CLI testing in installer
21. Solve `STRICT_REALTIME` bug in `FlowController`
22. Fix SEGV in `PDPServer` destructor
23. Fixes in boost interprocess
24. Fix submodules
25. Decrease `FASTDDS_GEN_API` and fixes in Mac compilation
26. Fix workaround for the C library names in generated code
27. Fix `ChainingTransport` implementing `fillMulticastLocator` and fixes in Low Bandwidth Transports
28. Remove instance samples when acknowledging unregister change
29. Include in `SampleInfo` change kind associated to taken sample
30. Install compiler `PDB` of the static library on Windows

This release includes the following **improvements**:

1. Reduce extra send in Data(p) traffic for discovery server
2. Remove Read the Docs related `CMake` instructions
3. Remove statistics network submessage and remove `FASTDDS_STATISTICS` compile option (always ON)
4. Set max ASIO buffers based on `IOV_MAX`
5. Remove `versions.md` in private repository
6. Topic Interest Discovery Server refactor
7. Add a public API to validate the features of a license too
8. Use wildcard of topic interest in `SUPER_CLIENTs`
9. Improve default TCP configuration
10. Improve discovery performance
11. Bundle `foonathan_memory` in thirdparty instead of depending on `foonathan_memory_vendor`
12. Adapt `foonathan_memory` to FACE restrictions
13. Idl parser bitsets followup
14. Tag the Fast DDS Pro only API with the `fastdds_pro` Doxygen command
15. Portability fixes for restricted standard libraries
16. Option to leave out wide characters and strings
17. Add custom buffers example
18. Option to avoid standard thread support and atomic operations
19. Workarounds for restricted standard libraries
20. Options to leave out what restricted platforms cannot have
21. Uncrustify dusting
22. Regenerate types with Fast DDS Gen 4.5.0.1

This release includes the following **CI improvements**:

1. Add backport workflow to Fast DDS Pro
2. Fix pass of `use-ccache` when running with dispatch
3. Use Pro Token for pushing backports
4. Add `TSan` workflow to Fast DDS Pro
5. Use Github App instead of token for backports
6. Fix typo in workflow step name
7. Fix GCC-latest build step
8. Bring Discovery Server testing framework to Fast DDS Pro
9. Improve `UBSan` and `ASan` CI jobs: merge jobs and not generate artifacts
10. Automatically add milestones to backports
11. Restore tests CMakeLists.txt
12. Build the blackbox tests without `SQLite`
13. Refactor ABI checker
14. Enable test suite filtering
15. Use merge branch in ABI checker and `-Og` optimization
16. Optimize unittest build time
17. Add test for nesting depth limit in type deserialization
18. Get Fast-DDS-Gen-Pro in Windows workflow
19. Solve DDSContentFilter.WithFragmentation flakiness
20. Add alternative build that uses the listed submodules

#### SEE ALSO
For further information about the corresponding versions of other products related to this Fast DDS version, please
refer to the [eProsima products compatibility](versions.md#dependencies-compatibilities-product-compatibility) section.

#### SEE ALSO
Information about the release lifecycle can be found [here](https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md).
