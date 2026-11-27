<a id="python-api-pim-livelinesschangedstatus"></a>

<a id="livelinesschangedstatus"></a>

# LivelinessChangedStatus

### *class* fastdds.LivelinessChangedStatus

A struct storing the liveliness changed status

#### *property* alive_count

The total number of currently active publishers that write the topic read by the subscriber This count increases when a newly matched publisher asserts its liveliness for the first time or when a publisher previously considered to be not alive reasserts its liveliness. The count decreases when a publisher considered alive fails to assert its liveliness and becomes not alive, whether because it was deleted normally or for some other reason

#### *property* alive_count_change

The change in the alive_count since the last time the listener was called or the status was read

#### *property* last_publication_handle

Handle to the last publisher whose change in liveliness caused this status to change

#### *property* not_alive_count

The total count of current publishers that write the topic read by the subscriber that are no longer asserting their liveliness This count increases when a publisher considered alive fails to assert its liveliness and becomes not alive for some reason other than the normal deletion of that publisher. It decreases when a previously not alive publisher either reasserts its liveliness or is deleted normally

#### *property* not_alive_count_change

The change in the not_alive_count since the last time the listener was called or the status was read

#### *property* thisown

The membership flag
