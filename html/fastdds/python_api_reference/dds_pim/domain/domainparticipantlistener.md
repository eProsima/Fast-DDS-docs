<a id="python-api-pim-domainparticipantlistener"></a>

<a id="domainparticipantlistener"></a>

# DomainParticipantListener

### *class* fastdds.DomainParticipantListener

Class DomainParticipantListener, overrides behaviour towards certain events.

#### on_data_reader_discovery(participant, reason, info, should_be_ignored)

This method is called when a new DataReader is discovered, or a previously discovered DataReader changes
its QOS or is removed.

* **Parameters:**
  * **[****in****]** – participant        Pointer to the Participant which discovered the remote reader.
  * **[****in****]** – reason             The reason motivating this method to be called.
  * **[****in****]** – info               Remote reader information.
  * **[****out****]** – should_be_ignored  Flag to indicate the library to automatically ignore the discovered reader.

#### on_data_writer_discovery(participant, reason, info, should_be_ignored)

This method is called when a new DataWriter is discovered, or a previously discovered DataWriter changes
its QOS or is removed.

* **Parameters:**
  * **[****in****]** – participant        Pointer to the Participant which discovered the remote writer.
  * **[****in****]** – reason             The reason motivating this method to be called.
  * **[****in****]** – info               Remote writer information.
  * **[****out****]** – should_be_ignored  Flag to indicate the library to automatically ignore the discovered writer.

#### on_participant_discovery(participant, reason, info, should_be_ignored)

This method is called when a new Participant is discovered, or a previously discovered participant changes
its QOS or is removed.

* **Parameters:**
  * **[****out****]** – participant Pointer to the Participant which discovered the remote participant.
  * **[****out****]** – reason Reason of the change in the status of the discovered participant.
  * **[****out****]** – info Remote participant information. User can take ownership of the object.
  * **[****out****]** – should_be_ignored Flag to indicate the library to automatically ignore the discovered Participant.

#### should_endpoints_match(participant, reader_info, writer_info)

This method is called to check whether a discovered DataReader and DataWriter should be matched or not.
It is called after successful validation of the standard DDS matching rules.
By default, they will match, but the user can override this method to implement custom matching logic.

* **Parameters:**
  * **[****in****]** – participant Pointer to the Participant which discovered the remote endpoints.
  * **[****in****]** – reader_info Remote reader information.
  * **[****in****]** – writer_info Remote writer information.
* **Return type:**
  boolean
* **Returns:**
  true if the endpoints should be matched, false otherwise.

#### *property* thisown

The membership flag
