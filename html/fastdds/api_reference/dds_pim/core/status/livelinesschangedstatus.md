<a id="api-pim-livelinesschangedstatus"></a>

<a id="livelinesschangedstatus"></a>

# LivelinessChangedStatus

### struct LivelinessChangedStatus

A struct storing the liveliness changed status. 

### Public Members

### std::int32_t alive_count = 0

The total number of currently active publishers that write the topic read by the subscriber. 

This count increases when a newly matched publisher asserts its liveliness for the first time or when a publisher previously considered to be not alive reasserts its liveliness. The count decreases when a publisher considered alive fails to assert its liveliness and becomes not alive, whether because it was deleted normally or for some other reason 

### std::int32_t not_alive_count = 0

The total count of current publishers that write the topic read by the subscriber that are no longer asserting their liveliness. 

This count increases when a publisher considered alive fails to assert its liveliness and becomes not alive for some reason other than the normal deletion of that publisher. It decreases when a previously not alive publisher either reasserts its liveliness or is deleted normally 

### std::int32_t alive_count_change = 0

The change in the alive_count since the last time the listener was called or the status was read. 

### std::int32_t not_alive_count_change = 0

The change in the not_alive_count since the last time the listener was called or the status was read. 

### InstanceHandle_t last_publication_handle

Handle to the last publisher whose change in liveliness caused this status to change.
