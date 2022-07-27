.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_core_waitsets:

Conditions and Wait-sets
========================

Conditions (in conjunction with wait-sets) provide an alternative mechanism to allow the middleware
to notify communication status changes (including arrival of data) to the application.

This mechanism is wait-based. Its general use pattern is as follows:

* The application indicates which relevant information it wants to get, by means of :ref:`api_pim_condition`
  objects (:ref:`api_pim_guardcondition`, :ref:`api_pim_statuscondition`, or :ref:`api_pim_readcondition`)
  and attaching them to a :ref:`api_pim_waitset` via the |WaitSet::attach_condition-api| call.
* It then waits on that :ref:`api_pim_waitset` via the |WaitSet::wait-api|
  call until the trigger value of one or several :ref:`api_pim_condition` objects become true.
* It then uses the result of the |WaitSet::wait-api| (i.e., the list of :ref:`api_pim_condition` objects with
  trigger_value == true) to actually get the information by calling:

  * |Entity::get_status_changes-api|, then checking if any of the changes is relevant using the |StatusMask::is_active-api|
    method on the result and finally calling get_<communication_status> on the relevant Entity, when the condition is a StatusCondition and the status changes refer to plain communication status.
    Refer to :ref:`dds_layer_core_status` for additional information on the different statuses that can be queried.
  * |Entity::get_status_changes-api| and then |Subscriber::get_datareaders-api| on the relevant Subscriber, when the condition is a
    StatusCondition and the status changes refer to DataOnReaders.
  * |Entity::get_status_changes-api| and then |DataReader::read-api|/|DataReader::take-api| on the relevant DataReader, when the condition is a
    StatusCondition and the status changes refer to DataAvailable.
  * Directly |DataReader::read_w_condition-api|/|DataReader::take_w_condition-api| on the DataReader with the :ref:`api_pim_condition` as a parameter,
    when it is a ReadCondition

* When a Condition is no longer relevant it can be detached from a :ref:`api_pim_waitset` via the
  |WaitSet::detach_condition-api| call.

The first step is usually done in an initialization phase, while the others are put in the
application main loop.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_WAITSET_EXAMPLE
   :end-before: //!
   :dedent: 4

Calling the |WaitSet::wait-api| operation on the :ref:`api_pim_waitset` will block the calling thread
if the trigger value of all the conditions attached to it are false.
The thread will wake up, and the |WaitSet::wait-api| operation will return RETCODE_OK, whenever the trigger value of any
of the attached conditions becomes true.

GuardCondition
--------------
A condition for which the trigger value is completely controlled by the application via its
|GuardCondition::set_trigger_value-api| operation.

StatusCondition
---------------
A condition that triggers whenever there are changes on the communication statuses of an Entity.

The sensitivity of the :ref:`api_pim_statuscondition` to a particular communication status is controlled by the
list of enabled_statuses set on the condition by means of the |StatusCondition::set_enabled_statuses-api| operation.

ReadCondition
-------------
A condition that triggers whenever the DataReader that created it contains at least a sample with
SampleState, ViewState, and InstanceState matching those of the ReadCondition.

The fact that the trigger value of a ReadCondition is dependent on the presence of samples on the
associated DataReader implies that a single take operation can potentially change the trigger value
of several ReadCondition conditions.
For example, if all samples are taken, any ReadCondition associated with the DataReader that were
triggered before, will see their trigger value changed to false.
Note that this does not guarantee that WaitSet objects that were separately attached to those
conditions will not be woken up.
Once we have trigger_value == true on a condition, it may wake up the attached :ref:`api_pim_waitset`.
The condition transitioning to trigger_value == false does not necessarily ‘unwakeup’ the :ref:`api_pim_waitset`,
as ‘unwakening’ may not be possible in general.
The consequence is that an application blocked on a :ref:`api_pim_waitset` may return from the wait with a list
of conditions, some of which are no longer triggered.
This also may be the consequence of user actions.
A user manually calling |GuardCondition::set_trigger_value-api| could potentially trigger the same behavior.
This is unavoidable if multiple threads are concurrently waiting on separate :ref:`api_pim_waitset` objects and
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

