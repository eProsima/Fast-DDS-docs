<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-topic"></a>

<a id="topic"></a>

# Topic

A Topic conceptually fits between publications and subscriptions.
Each publication channel must be unambiguously identified by the subscriptions in order to receive only the data flow
they are interested in, and not data from other publications.
A Topic serves this purpose, allowing publications and subscriptions that share the same
Topic to match and start communicating.
In that sense, the Topic acts as a description for a data flow.

Publications are always linked to a single [Topic](topic/topic.md#dds-layer-topic-topic), while subscriptions are linked to a
broader concept of [TopicDescription](topicDescription/topicDescription.md#dds-layer-topic-topicdescription).

![image](01-figures/topic_class_diagram.svg)

*Topic class diagram*

* [Topics, keys and instances](instances.md)
  * [Instance advantages](instances.md#instance-advantages)
  * [Instance lifecycle](instances.md#instance-lifecycle)
  * [Practical applications](instances.md#practical-applications)
    * [Commercial flights tracking](instances.md#commercial-flights-tracking)
    * [Relational databases](instances.md#relational-databases)
* [TopicDescription](topicDescription/topicDescription.md)
* [Topic](topic/topic.md)
  * [TopicQos](topic/topic.md#topicqos)
    * [Default TopicQos](topic/topic.md#default-topicqos)
* [ContentFilteredTopic](contentFilteredTopic/contentFilteredTopic.md)
* [TopicListener](topicListener/topicListener.md)
* [Definition of data types](typeSupport/typeSupport.md)
  * [Dynamic data types](typeSupport/typeSupport.md#dynamic-data-types)
  * [Data types with a key](typeSupport/typeSupport.md#data-types-with-a-key)
  * [Type support context](typeSupport/typeSupport.md#type-support-context)
* [Creating a Topic](topic/createTopic.md)
  * [Profile based creation of a Topic](topic/createTopic.md#profile-based-creation-of-a-topic)
  * [Deleting a Topic](topic/createTopic.md#deleting-a-topic)
* [Filtering data on a Topic](contentFilteredTopic/createContentFilteredTopic.md)
  * [Creating a ContentFilteredTopic](contentFilteredTopic/createContentFilteredTopic.md#creating-a-contentfilteredtopic)
  * [Updating the filter expression and parameters](contentFilteredTopic/createContentFilteredTopic.md#updating-the-filter-expression-and-parameters)
  * [Deleting a ContentFilteredTopic](contentFilteredTopic/createContentFilteredTopic.md#deleting-a-contentfilteredtopic)
* [The default SQL-like filter](contentFilteredTopic/defaultFilter.md)
  * [Grammar](contentFilteredTopic/defaultFilter.md#grammar)
  * [Like condition](contentFilteredTopic/defaultFilter.md#like-condition)
  * [Match condition](contentFilteredTopic/defaultFilter.md#match-condition)
  * [Type comparisons](contentFilteredTopic/defaultFilter.md#type-comparisons)
  * [Example](contentFilteredTopic/defaultFilter.md#example)
* [Using custom filters](contentFilteredTopic/customFilters.md)
  * [Creating the Custom Filter](contentFilteredTopic/customFilters.md#creating-the-custom-filter)
  * [Creating the Factory for the Custom Filter](contentFilteredTopic/customFilters.md#creating-the-factory-for-the-custom-filter)
  * [Registering the Factory](contentFilteredTopic/customFilters.md#registering-the-factory)
  * [Creating a ContentFilteredTopic using the Custom Filter](contentFilteredTopic/customFilters.md#creating-a-contentfilteredtopic-using-the-custom-filter)
* [Where is filtering applied: writer vs reader side](contentFilteredTopic/writerFiltering.md)
  * [Conditions for writer side filtering](contentFilteredTopic/writerFiltering.md#conditions-for-writer-side-filtering)
  * [Discovery race condition](contentFilteredTopic/writerFiltering.md#discovery-race-condition)
  * [Writer side filtering on late joiners <span class="sd-badge sd-outline-primary sd-text-primary" title="Exclusive to Fast DDS Pro">Pro</span>](contentFilteredTopic/writerFiltering.md#writer-side-filtering-on-late-joiners-pro)
* [Fast DDS-Gen for data types source code generation](fastddsgen/fastddsgen.md)
  * [Basic usage](fastddsgen/fastddsgen.md#basic-usage)
  * [Output files](fastddsgen/fastddsgen.md#output-files)
