<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-topic-contentfilteredtopic-custom-filters"></a>

<a id="using-custom-filters"></a>

# Using custom filters

Fast DDS API supports the creation and later registration of user’s custom filters to be used in the creation of a
[`ContentFilteredTopic`](../../../api_reference/dds_pim/topic/contentfilteredtopic.md#_CPPv4N8eprosima7fastdds3dds20ContentFilteredTopicE).
Required steps for using a Custom Filter are:

- [Creating the Custom Filter](#dds-layer-topic-customfilter-creation)
- [Creating the Factory for the Custom Filter](#dds-layer-topic-customfilter-factory)
- [Registering the Factory](#dds-layer-topic-customfilter-register)
- [Creating a ContentFilteredTopic using the Custom Filter](#dds-layer-topic-customfilter-create-topic)

<a id="dds-layer-topic-customfilter-creation"></a>

<a id="creating-the-custom-filter"></a>

## Creating the Custom Filter

A custom filter must be implemented by a class which inherits from [`IContentFilter`](../../../api_reference/dds_pim/topic/icontentfilter.md#_CPPv4N8eprosima7fastdds3dds14IContentFilterE).
Only one function must be implemented, overriding [`evaluate()`](../../../api_reference/dds_pim/topic/icontentfilter.md#_CPPv4NK8eprosima7fastdds3dds14IContentFilter8evaluateERK17SerializedPayloadRK16FilterSampleInfoRK6GUID_t).
Each time a sample is received by a [`DataReader`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE), this function is called with next arguments.

- `payload` - The serialized payload of the sample which the custom filter has to evaluate.
- `sample_info` - The extra information which accompanies the sample.
- `reader_guid` - The GUID of the reader for which the filter is being evaluated.

The function returns a boolean where `true` implies the sample is accepted and `false` rejects the sample.

Next snippet code shows an example of Custom Filter which deserialize the `index` field from a serialized sample and
rejects samples where `index` > `low_mark_` and `index` < `high_mark_`.

```c++
class MyCustomFilter : public IContentFilter
{
public:

    MyCustomFilter(
            int low_mark,
            int high_mark)
        : low_mark_(low_mark)
        , high_mark_(high_mark)
    {
    }

    bool evaluate(
            const SerializedPayload& payload,
            const FilterSampleInfo& sample_info,
            const GUID_t& reader_guid) const override
    {
        // Deserialize the `index` field from the serialized sample.
        /* IDL
         *
         * struct HelloWorld
         * {
         *     long index;
         *     string message;
         * }
         */
        eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload.data), payload.length);
        eprosima::fastcdr::Cdr deser(fastbuffer);
        // Deserialize encapsulation.
        deser.read_encapsulation();
        int index = 0;

        // Deserialize `index` field.
        try
        {
            deser >> index;
        }
        catch (eprosima::fastcdr::exception::NotEnoughMemoryException& exception)
        {
            return false;
        }

        // Custom filter: reject samples where index > low_mark_ and index < high_mark_.
        if (index > low_mark_ && index < high_mark_)
        {
            return false;
        }

        return true;
    }

private:

    int low_mark_ = 0;
    int high_mark_ = 0;

};
```

<a id="dds-layer-topic-customfilter-factory"></a>

<a id="creating-the-factory-for-the-custom-filter"></a>

## Creating the Factory for the Custom Filter

Fast DDS creates filters through a factory.
Therefore a factory which provides instantiating of a Custom Filter must be implemented.

A Custom Filter’s factory has to inherit from [`IContentFilterFactory`](../../../api_reference/dds_pim/topic/icontentfilterfactory.md#_CPPv4N8eprosima7fastdds3dds21IContentFilterFactoryE).
This interface requires two functions to be implemented.

Each time a Custom Filter has to be created or updated, [`create_contentfilteredtopic()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant27create_contentfilteredtopicERKNSt6stringEP5TopicRKNSt6stringERKNSt6vectorINSt6stringEEER12ReturnCode_t) calls
internally [`create_content_filter()`](../../../api_reference/dds_pim/topic/icontentfilterfactory.md#_CPPv4N8eprosima7fastdds3dds21IContentFilterFactory21create_content_filterEPKcPKcPK13TopicDataTypePKcRK12ParameterSeqRP14IContentFilter) with these arguments:

- `filter_class_name` - Filter class name for which the factory is being called.
  It allows using the same factory for different filter classes.
- `type_name` - Type name of the topic being filtered.
- `data_type` - Type support object of the topic being filtered.
- `filter_expression` - Custom filter expression.
- `filter_parameters` - Values to set for the filter parameters (where custom filter expression has its pattern to
  substitute them).
- `filter_instance` - When a filter is being created, it will be `nullptr` on input, and will have the pointer to
  the created filter instance on output.
  When a filter is being updated, it will have a previously returned pointer on input.

This function should return the result of the operation.

When a Custom Filter should be removed, [`delete_contentfilteredtopic()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant27delete_contentfilteredtopicEPK20ContentFilteredTopic) calls internally
[`delete_content_filter()`](../../../api_reference/dds_pim/topic/icontentfilterfactory.md#_CPPv4N8eprosima7fastdds3dds21IContentFilterFactory21delete_content_filterEPKcP14IContentFilter).
The factory must remove the provided Custom Filter’s instance.

Next snippet code shows an example of Custom Filter’s factory which manages instances of the Custom Filter implemented
in the previous section.

```c++
class MyCustomFilterFactory : public IContentFilterFactory
{
public:

    ReturnCode_t create_content_filter(
            const char* filter_class_name, // My custom filter class name is 'MY_CUSTOM_FILTER'.
            const char* type_name, // This custom filter only supports one type: 'HelloWorld'.
            const TopicDataType* /*data_type*/, // Not used in this implementation.
            const char* filter_expression, // This Custom Filter doesn't implement a filter expression.
            const ParameterSeq& filter_parameters, // Always need two parameters to be set: low_mark and high_mark.
            IContentFilter*& filter_instance) override
    {
        // Check the ContentFilteredTopic should be created by my factory.
        if (0 != strcmp(filter_class_name, "MY_CUSTOM_FILTER"))
        {
            return RETCODE_BAD_PARAMETER;
        }

        // Check the ContentFilteredTopic is created for the unique type this Custom Filter supports.
        if (0 != strcmp(type_name, "HelloWorld"))
        {
            return RETCODE_BAD_PARAMETER;
        }

        // Check that the two mandatory filter parameters are set.
        if (2 != filter_parameters.length())
        {
            return RETCODE_BAD_PARAMETER;
        }

        // If there is an update, delete previous instance.
        if (nullptr != filter_instance)
        {
            delete(dynamic_cast<MyCustomFilter*>(filter_instance));
        }

        // Instantiation of the Custom Filter.
        filter_instance = new MyCustomFilter(std::stoi(filter_parameters[0]), std::stoi(filter_parameters[1]));

        return RETCODE_OK;
    }

    ReturnCode_t delete_content_filter(
            const char* filter_class_name,
            IContentFilter* filter_instance) override
    {
        // Check the ContentFilteredTopic should be created by my factory.
        if (0 != strcmp(filter_class_name, "MY_CUSTOM_FILTER"))
        {
            return RETCODE_BAD_PARAMETER;
        }

        // Deletion of the Custom Filter.
        delete(dynamic_cast<MyCustomFilter*>(filter_instance));

        return RETCODE_OK;
    }

};
```

<a id="dds-layer-topic-customfilter-register"></a>

<a id="registering-the-factory"></a>

## Registering the Factory

To be able to use the Custom Filter in an application, the Custom Filter’s factory must be registered in the
[`DomainParticipant`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE).
Next snippet code shows how to register a factory through API function
[`register_content_filter_factory()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant31register_content_filter_factoryEPKcPC21IContentFilterFactory).

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Create Custom Filter Factory
MyCustomFilterFactory* factory = new MyCustomFilterFactory();


// Registration of the factory
if (RETCODE_OK !=
        participant->register_content_filter_factory("MY_CUSTOM_FILTER", factory))
{
    // Error
    return;
}
```

<a id="dds-layer-topic-customfilter-create-topic"></a>

<a id="creating-a-contentfilteredtopic-using-the-custom-filter"></a>

## Creating a ContentFilteredTopic using the Custom Filter

[Creating a ContentFilteredTopic](createContentFilteredTopic.md#dds-layer-topic-contentfilteredtopic-creation) explains how to create a [`ContentFilteredTopic`](../../../api_reference/dds_pim/topic/contentfilteredtopic.md#_CPPv4N8eprosima7fastdds3dds20ContentFilteredTopicE).
In the case of using a Custom Filter, [`create_contentfilteredtopic()`](../../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant27create_contentfilteredtopicERKNSt6stringEP5TopicRKNSt6stringERKNSt6vectorINSt6stringEEER12ReturnCode_t) has an overload adding an
argument to select the Custom Filter.

Next snippet code shows how to create a [`ContentFilteredTopic`](../../../api_reference/dds_pim/topic/contentfilteredtopic.md#_CPPv4N8eprosima7fastdds3dds20ContentFilteredTopicE) using the Custom Filter.

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Create the Topic.
Topic* topic =
        participant->create_topic("HelloWorldTopic", "HelloWorld", TOPIC_QOS_DEFAULT);
if (nullptr == topic)
{
    // Error
    return;
}

// Create a ContentFilteredTopic selecting the Custom Filter and using no expression with two parameters
// Filter expression cannot be an empty one even when it is not used by the custom filter, as that effectively
// disables any filtering
std::string expression = " ";
std::vector<std::string> parameters;
parameters.push_back("10"); // Parameter for low_mark
parameters.push_back("20"); // Parameter for low_mark
ContentFilteredTopic* filter_topic =
        participant->create_contentfilteredtopic("HelloWorldFilteredTopic1", topic, expression, parameters,
                "MY_CUSTOM_FILTER");
if (nullptr == filter_topic)
{
    // Error
    return;
}

// The ContentFilteredTopic instances can then be used to create DataReader objects.
Subscriber* subscriber =
        participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
if (nullptr == subscriber)
{
    // Error
    return;
}

DataReader* reader_on_filter = subscriber->create_datareader(filter_topic, DATAREADER_QOS_DEFAULT);
if (nullptr == reader_on_filter)
{
    // Error
    return;
}
```

#### IMPORTANT
Even though this specific custom filtering example is not using the filter expression, mind that the expression
cannot be an empty string as that disables filtering as explained in
[Creating a ContentFilteredTopic](createContentFilteredTopic.md#dds-layer-topic-contentfilteredtopic-creation).

#### NOTE
Deleting a ContentFilteredTopic which uses a Custom Filter is done exactly in the same manner explained in
[Deleting a ContentFilteredTopic](createContentFilteredTopic.md#dds-layer-topic-contentfilteredtopic-deletion).
