<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="release-notes-basic"></a>

<a id="id1"></a>

# [Version 3.6.3](https://fast-dds.docs.eprosima.com/en/v3.6.3/index.html)

This release includes the following **security fixes**:

1. [CVE-2025-67108](https://www.cve.org/CVERecord?id=CVE-2025-67108)
2. [CVE-2026-45098](https://www.cve.org/CVERecord?id=CVE-2026-45098)
3. [CVE-2026-49861](https://www.cve.org/CVERecord?id=CVE-2026-49861)
4. [CVE-2026-49862](https://www.cve.org/CVERecord?id=CVE-2026-49862)
5. [CVE-2026-49863](https://www.cve.org/CVERecord?id=CVE-2026-49863)
6. [CVE-2026-53588](https://www.cve.org/CVERecord?id=CVE-2026-53588)
7. [CVE-2026-53589](https://www.cve.org/CVERecord?id=CVE-2026-53589)
8. [CVE-2026-53590](https://www.cve.org/CVERecord?id=CVE-2026-53590)
9. [CVE-2026-55063](https://www.cve.org/CVERecord?id=CVE-2026-55063)
10. [CVE-2026-102681](https://www.cve.org/CVERecord?id=CVE-2026-102681)
11. [GHSA-f5r5-m785-f5wg](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-f5r5-m785-f5wg)
12. [GHSA-w92j-25c7-68qx](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-w92j-25c7-68qx)
13. [GHSA-33xc-m88x-3cp8](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-33xc-m88x-3cp8)
14. [GHSA-f5r6-qvhq-mx93](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-f5r6-qvhq-mx93)
15. [GHSA-wfcj-cv96-9wj9](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-wfcj-cv96-9wj9)
16. [GHSA-g355-8h7f-6w8v](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-g355-8h7f-6w8v)
17. [GHSA-6pwr-vh77-2x8m](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-6pwr-vh77-2x8m)
18. [GHSA-58x7-8g6g-mpp8](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-58x7-8g6g-mpp8)
19. [GHSA-f6cw-h682-xw3p](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-f6cw-h682-xw3p)
20. [GHSA-mg54-9fm8-xvf3](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-mg54-9fm8-xvf3)
21. [GHSA-j4xc-fjx8-wfg4](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-j4xc-fjx8-wfg4)
22. [GHSA-57rp-jm6r-xjw5](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-57rp-jm6r-xjw5)
23. [GHSA-f4hw-9j9g-j8xj](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-f4hw-9j9g-j8xj)
24. [GHSA-6m2p-wjv5-386v](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-6m2p-wjv5-386v)
25. [GHSA-5fgv-8wxq-3f69](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-5fgv-8wxq-3f69)
26. [GHSA-hhx6-8qfx-8c23](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-hhx6-8qfx-8c23)
27. [GHSA-w75r-338m-pc5x](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-w75r-338m-pc5x)
28. [GHSA-4r2x-mjm7-r285](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-4r2x-mjm7-r285)
29. [GHSA-vx4x-38rc-27w8](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-vx4x-38rc-27w8)
30. [GHSA-6prg-7f44-f36j](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-6prg-7f44-f36j)
31. [GHSA-m6f5-9h75-494p](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-m6f5-9h75-494p)
32. [GHSA-p8f8-79c6-vgvg](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-p8f8-79c6-vgvg)
33. [GHSA-3m58-hxq8-985j](https://github.com/eProsima/Fast-DDS/security/advisories/GHSA-3m58-hxq8-985j)

This release includes the following **fixes**:

1. Catch exception when constructing a payload pool Node
2. Tests and fixes for type support context-aware operations
3. Implement payload release in move assignment
4. Fix datasharing infinite loop
5. Account for padding when protecting payloads with `GMAC` only
6. Fix shared-memory segment crash on port open
7. Fix minor data races
8. Fix `TSAN` report warnings for interleaved logs
9. Fix build of docs after HTML refactor
10. Fix leak in UDP when opening input channels
11. Fix volatile reader desync after initial positioning `GAP`
12. Set max `ASIO` buffers based on `IOV_MAX`
13. Fix a race that lets the payload pool registry hand out a null pool
14. Fix UDP send timeout for durations of one second or longer
15. Release topic payload pool registry entries when their pools are released
16. Validate shared-memory index links and check segment creation
17. Avoid a 333 ms wait when opening a healthy shared memory port
18. Fix `FTBFS` against `OpenSSL 4.0`
19. Keep room in `ReaderHistory` for missing changes
20. Include `OpenSSL` version header before version checks
21. Solve `STRICT_REALTIME` bug in `FlowController`
22. Fix default history depth in examples
23. Store the grant only once it has been domain filtered
24. Do not acknowledge intraprocess `DATA` from an unmatched writer
25. Check path validity in IDL Parser

This release includes the following **improvements**:

1. Update Release Support with `v3.2.5` & `v2.6.12`
2. Rename Fast DDS Monitor to DDS Monitor
3. Updates after `v2.14.7` release
4. Use the `SPDX` identifier `Apache-2.0` in `package.xml` license tag
5. Custom security policy
6. Improve default TCP configuration
7. Add XML support for `DataRepresentationQosPolicy`
8. Update Fast CDR to `2.3.7`
9. Update Fast CDR to `2.3.8`
10. Regenerate types with Fast DDS Gen `4.3.1`
11. Add OSS fuzz harness for SEDP/SPDP CDR deserializers

This release includes the following **CI improvements**:

1. Fix `SEGV` in latency test
2. Enable `UBSan` in nightlies
3. Avoid using `latency_data_in_` when using loans in `LatencyTestPublisher`
4. Fix typo in workflow step name
5. Fix CLI testing in installer
6. Optimize unittest build time

#### SEE ALSO
For further information about the corresponding versions of other products related to this Fast DDS version, please
refer to the [eProsima products compatibility](versions.md#dependencies-compatibilities-product-compatibility) section.

#### SEE ALSO
Information about the release lifecycle can be found [here](https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md).
