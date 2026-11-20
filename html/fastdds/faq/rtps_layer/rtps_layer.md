<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="freq-rtps-layer-questions"></a>

<a id="rtps-layer-frequently-asked-questions"></a>

# RTPS LAYER Frequently Asked Questions

**What is the primary function of the RTPS Layer in eprosima Fast DDS, and how does it differ from the DDS Layer?**

<br/> The lower level RTPS Layer of eprosima Fast DDS serves as an implementation of the protocol defined in the RTPS standard.
This layer provides more control over the internals of the communication protocol than the DDS Layer, so advanced users have finer control over the library’s functionalities.
For further information, refer to [RTPS Layer](../../rtps_layer/rtps_layer.md#rtps-layer).

---

**What is the primary responsibility of an "RTPSParticipant" in the context of the RTPS Layer?**

<br/> As the RTPS standard specifies, `RTPSWriters` and `RTPSReaders` are always associated with a `History` element.
In the DDS Layer, its creation and management is hidden, but in the RTPS Layer, you have full control over its creation and configuration.
For further information, refer to [RTPS Layer](../../rtps_layer/rtps_layer.md#rtps-layer).

---

**What is the purpose of creating a "WriterHistory" when creating an RTPS Writer?**

<br/> Writers are created with [`RTPSDomain::createRTPSWriter()`](../../api_reference/rtps/RTPSDomain.md#_CPPv4N8eprosima7fastdds4rtps10RTPSDomain16createRTPSWriterEP15RTPSParticipantR16WriterAttributesP13WriterHistoryP14WriterListener) and configured with a `WriterAttributes` structure.
They also need a `WriterHistory` which is configured with a `HistoryAttributes` structure.
For further information, refer to [RTPS Layer](../../rtps_layer/rtps_layer.md#rtps-layer).

---

**What is the purpose of using the "History" element in the RTPS Layer?**

<br/> In the RTPS Protocol, Readers and Writers save the data about a topic in their associated Histories.
Each piece of data is represented by a Change, which *eprosima Fast DDS* implements as `CacheChange_t`.
Changes are always managed by the History.
For further information, refer to [RTPS Layer](../../rtps_layer/rtps_layer.md#rtps-layer).

---

**How can a custom Payload Pool improve the performance of Writers and Readers in RTPS, and what should be considered when implementing one?**

<br/> A custom payload pool can improve the performance of writers and readers in RTPS by optimizing memory usage and reducing costly memory allocation operations, especially when dealing with large or variable-sized data.
When implementing one, it is essential to ensure that the payload size accommodates the serialized user data plus metadata, and to choose a strategy (e.g., preallocated, dynamic) that balances memory usage and allocation efficiency based on application needs.
For further information, refer to [Using a custom Payload Pool](../../rtps_layer/rtps_layer.md#rtps-layer-custom-payload-pool).

<br/>
