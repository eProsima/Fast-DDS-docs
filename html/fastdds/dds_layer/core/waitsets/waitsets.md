<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-core-waitsets"></a>

<a id="conditions-and-wait-sets"></a>

# Conditions and Wait-sets

Conditions (in conjunction with wait-sets) provide an alternative mechanism to allow the middleware
to notify communication status changes (including arrival of data) to the application.

This mechanism is wait-based. Its general use pattern is as follows:

* The application indicates which relevant information it wants to get, by means of [Condition](../../../api_reference/dds_pim/core/condition/condition.md#api-pim-condition)
  objects ([GuardCondition](../../../api_reference/dds_pim/core/condition/guardcondition.md#api-pim-guardcondition), [StatusCondition](../../../api_reference/dds_pim/core/condition/statuscondition.md#api-pim-statuscondition), or [ReadCondition](../../../api_reference/dds_pim/subscriber/readcondition.md#api-pim-readcondition))
  and attaching them to a [Wait-set](../../../api_reference/dds_pim/core/condition/waitset.md#api-pim-waitset) via the [`attach_condition()`](../../../api_reference/dds_pim/core/condition/waitset.md#_CPPv4N8eprosima7fastdds3dds7WaitSet16attach_conditionERK9Condition) call.
* It then waits on that [Wait-set](../../../api_reference/dds_pim/core/condition/waitset.md#api-pim-waitset) via the [`wait()`](../../../api_reference/dds_pim/core/condition/waitset.md#_CPPv4NK8eprosima7fastdds3dds7WaitSet4waitER12ConditionSeqKN3dds10Duration_tE)
  call until the trigger value of one or several [Condition](../../../api_reference/dds_pim/core/condition/condition.md#api-pim-condition) objects become true.
* It then uses the result of the [`wait()`](../../../api_reference/dds_pim/core/condition/waitset.md#_CPPv4NK8eprosima7fastdds3dds7WaitSet4waitER12ConditionSeqKN3dds10Duration_tE) (i.e., the list of [Condition](../../../api_reference/dds_pim/core/condition/condition.md#api-pim-condition) objects with
  trigger_value == true) to actually get the information by calling:
  * [`get_status_changes()`](../../../api_reference/dds_pim/core/entity.md#_CPPv4NK8eprosima7fastdds3dds6Entity18get_status_changesEv), then checking if any of the changes is relevant using the [`StatusMask::is_active()`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4NK8eprosima7fastdds3dds10StatusMask9is_activeE10StatusMask)
    method on the result and finally calling get_<communication_status> on the relevant Entity, when the condition is a StatusCondition and the status changes refer to plain communication status.
    Refer to [Status](../status/status.md#dds-layer-core-status) for additional information on the different statuses that can be queried.
  * [`get_status_changes()`](../../../api_reference/dds_pim/core/entity.md#_CPPv4NK8eprosima7fastdds3dds6Entity18get_status_changesEv) and then [`Subscriber::get_datareaders()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4NK8eprosima7fastdds3dds10Subscriber15get_datareadersERNSt6vectorIP10DataReaderEE) on the relevant Subscriber, when the condition is a
    StatusCondition and the status changes refer to DataOnReaders.
  * [`get_status_changes()`](../../../api_reference/dds_pim/core/entity.md#_CPPv4NK8eprosima7fastdds3dds6Entity18get_status_changesEv) and then [`DataReader::read()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader4readER18LoanableCollectionR13SampleInfoSeqNSt7int32_tE15SampleStateMask13ViewStateMask17InstanceStateMask)/[`DataReader::take()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader4takeER18LoanableCollectionR13SampleInfoSeqNSt7int32_tE15SampleStateMask13ViewStateMask17InstanceStateMask) on the relevant DataReader, when the condition is a
    StatusCondition and the status changes refer to DataAvailable.
  * Directly [`DataReader::read_w_condition()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader16read_w_conditionER18LoanableCollectionR13SampleInfoSeqNSt7int32_tEP13ReadCondition)/[`DataReader::take_w_condition()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader16take_w_conditionER18LoanableCollectionR13SampleInfoSeqNSt7int32_tEP13ReadCondition) on the DataReader with the [Condition](../../../api_reference/dds_pim/core/condition/condition.md#api-pim-condition) as a parameter,
    when it is a ReadCondition
* When a Condition is no longer relevant it can be detached from a [Wait-set](../../../api_reference/dds_pim/core/condition/waitset.md#api-pim-waitset) via the
  [`detach_condition()`](../../../api_reference/dds_pim/core/condition/waitset.md#_CPPv4N8eprosima7fastdds3dds7WaitSet16detach_conditionERK9Condition) call.

The first step is usually done in an initialization phase, while the others are put in the
application main loop.

```c++
class ApplicationJob
{
    WaitSet wait_set_;
    GuardCondition terminate_condition_;
    std::thread thread_;

    void main_loop()
    {
        // Main loop is repeated until the terminate condition is triggered
        while (false == terminate_condition_.get_trigger_value())
        {
            // Wait for any of the conditions to be triggered
            ReturnCode_t ret_code;
            ConditionSeq triggered_conditions;
            ret_code = wait_set_.wait(triggered_conditions, eprosima::fastdds::dds::c_TimeInfinite);
            if (RETCODE_OK != ret_code)
            {
                // ... handle error
                continue;
            }

            // Process triggered conditions
            for (Condition* cond : triggered_conditions)
            {
                StatusCondition* status_cond = dynamic_cast<StatusCondition*>(cond);
                if (nullptr != status_cond)
                {
                    Entity* entity = status_cond->get_entity();
                    StatusMask changed_statuses = entity->get_status_changes();

                    // Process status. Liveliness changed and data available are depicted as an example
                    if (changed_statuses.is_active(StatusMask::liveliness_changed()))
                    {
                        std::cout << "Liveliness changed reported for entity "
                                  << entity->get_instance_handle()
                                  << std::endl;
                    }

                    if (changed_statuses.is_active(StatusMask::data_available()))
                    {
                        std::cout << "Data avilable on reader " << entity->get_instance_handle() << std::endl;

                        FooSeq data_seq;
                        SampleInfoSeq info_seq;
                        DataReader* reader = static_cast<DataReader*>(entity);

                        // Process all the samples until no one is returned
                        while (RETCODE_OK == reader->take(data_seq, info_seq,
                                LENGTH_UNLIMITED, ANY_SAMPLE_STATE,
                                ANY_VIEW_STATE, ANY_INSTANCE_STATE))
                        {
                            // Both info_seq.length() and data_seq.length() will have the number of samples returned
                            for (FooSeq::size_type n = 0; n < info_seq.length(); ++n)
                            {
                                // Only samples with valid data should be accessed
                                if (info_seq[n].valid_data &&
                                        reader->is_sample_valid(&data_seq[n], &info_seq[n]))
                                {
                                    // Process sample on data_seq[n]
                                }
                            }

                            // must return the loaned sequences when done processing
                            reader->return_loan(data_seq, info_seq);
                        }
                    }
                }
            }
        }
    }

public:

    ApplicationJob(
            const std::vector<DataReader*>& readers,
            const std::vector<DataWriter*>& writers)
    {
        // Add a GuardCondition, so we can signal the processing thread to stop
        wait_set_.attach_condition(terminate_condition_);

        // Add the status condition of every reader and writer
        for (DataReader* reader : readers)
        {
            wait_set_.attach_condition(reader->get_statuscondition());
        }
        for (DataWriter* writer : writers)
        {
            wait_set_.attach_condition(writer->get_statuscondition());
        }

        thread_ = std::thread(&ApplicationJob::main_loop, this);
    }

    ~ApplicationJob()
    {
        // Signal the GuardCondition to force the WaitSet to wake up
        terminate_condition_.set_trigger_value(true);
        // Wait for the thread to finish
        thread_.join();
    }

};

// Application initialization
ReturnCode_t ret_code;
std::vector<DataReader*> application_readers;
std::vector<DataWriter*> application_writers;

// Create the participant, topics, readers, and writers.
ret_code = create_dds_application(application_readers, application_writers);
if (RETCODE_OK != ret_code)
{
    // ... handle error
    return;
}

{
    ApplicationJob main_loop_thread(application_readers, application_writers);

    // ... wait for application termination signaling (signal handler, user input, etc)

    // ... Destructor of ApplicationJob takes care of stopping the processing thread
}

// Destroy readers, writers, topics, and participant
destroy_dds_application();
```

Calling the [`wait()`](../../../api_reference/dds_pim/core/condition/waitset.md#_CPPv4NK8eprosima7fastdds3dds7WaitSet4waitER12ConditionSeqKN3dds10Duration_tE) operation on the [Wait-set](../../../api_reference/dds_pim/core/condition/waitset.md#api-pim-waitset) will block the calling thread
if the trigger value of all the conditions attached to it are false.
The thread will wake up, and the [`wait()`](../../../api_reference/dds_pim/core/condition/waitset.md#_CPPv4NK8eprosima7fastdds3dds7WaitSet4waitER12ConditionSeqKN3dds10Duration_tE) operation will return RETCODE_OK, whenever the trigger value of any
of the attached conditions becomes true.

<a id="guardcondition"></a>

## GuardCondition

A condition for which the trigger value is completely controlled by the application via its
[`set_trigger_value()`](../../../api_reference/dds_pim/core/condition/guardcondition.md#_CPPv4N8eprosima7fastdds3dds14GuardCondition17set_trigger_valueEb) operation.

<a id="statuscondition"></a>

## StatusCondition

A condition that triggers whenever there are changes on the communication statuses of an Entity.

The sensitivity of the [StatusCondition](../../../api_reference/dds_pim/core/condition/statuscondition.md#api-pim-statuscondition) to a particular communication status is controlled by the
list of enabled_statuses set on the condition by means of the [`set_enabled_statuses()`](../../../api_reference/dds_pim/core/condition/statuscondition.md#_CPPv4N8eprosima7fastdds3dds15StatusCondition20set_enabled_statusesERK10StatusMask) operation.

<a id="dds-layer-core-readcondition"></a>

<a id="readcondition"></a>

## ReadCondition

A condition that triggers whenever the DataReader that created it contains at least a sample with
SampleState, ViewState, and InstanceState matching those of the ReadCondition.

The fact that the trigger value of a ReadCondition is dependent on the presence of samples on the
associated DataReader implies that a single take operation can potentially change the trigger value
of several ReadCondition conditions.
For example, if all samples are taken, any ReadCondition associated with the DataReader that were
triggered before, will see their trigger value changed to false.
Note that this does not guarantee that WaitSet objects that were separately attached to those
conditions will not be woken up.
Once we have trigger_value == true on a condition, it may wake up the attached [Wait-set](../../../api_reference/dds_pim/core/condition/waitset.md#api-pim-waitset).
The condition transitioning to trigger_value == false does not necessarily ‘unwakeup’ the [Wait-set](../../../api_reference/dds_pim/core/condition/waitset.md#api-pim-waitset),
as ‘unwakening’ may not be possible in general.
The consequence is that an application blocked on a [Wait-set](../../../api_reference/dds_pim/core/condition/waitset.md#api-pim-waitset) may return from the wait with a list
of conditions, some of which are no longer triggered.
This also may be the consequence of user actions.
A user manually calling [`set_trigger_value()`](../../../api_reference/dds_pim/core/condition/guardcondition.md#_CPPv4N8eprosima7fastdds3dds14GuardCondition17set_trigger_valueEb) could potentially trigger the same behavior.
This is unavoidable if multiple threads are concurrently waiting on separate [Wait-set](../../../api_reference/dds_pim/core/condition/waitset.md#api-pim-waitset) objects and
taking data associated with the same DataReader entity.

To elaborate further, consider the following example:
A ReadCondition that has a sample_state_mask = {NOT_READ} will have trigger_value == true whenever
a new sample arrives and will transition to false as soon as all the newly-arrived samples are
either read (so their status changes to READ) or taken (so they are no longer managed by the
DataReader).
However, if the same ReadCondition had a sample_state_mask = {READ, NOT_READ}, then the
trigger_value would only become false once all the newly-arrived samples are taken (it is not
sufficient to read them as that would only change the SampleState to READ which overlaps the mask
on the ReadCondition).
