<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="release-notes-basic"></a>

<a id="id1"></a>

# [Version 3.6.2](https://fast-dds.docs.eprosima.com/en/v3.6.2/index.html)

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

#### SEE ALSO
For further information about the corresponding versions of other products related to this Fast DDS version, please
refer to the [eProsima products compatibility](versions.md#dependencies-compatibilities-product-compatibility) section.

#### SEE ALSO
Information about the release lifecycle can be found [here](https://github.com/eProsima/Fast-DDS/blob/master/RELEASE_SUPPORT.md).
