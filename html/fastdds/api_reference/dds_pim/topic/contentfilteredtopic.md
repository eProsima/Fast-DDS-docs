<a id="api-pim-contentfilteredtopic"></a>

<a id="contentfilteredtopic"></a>

# ContentFilteredTopic

### class ContentFilteredTopic : public eprosima::fastdds::dds::[TopicDescription](topicdescription.md#_CPPv4N8eprosima7fastdds3dds16TopicDescriptionE)

Specialization of [TopicDescription](topicdescription.md#classeprosima_1_1fastdds_1_1dds_1_1TopicDescription) that allows for content-based subscriptions. 

### Public Functions

### [Topic](topic_class.md#_CPPv4N8eprosima7fastdds3dds5TopicE) \*get_related_topic() const

Getter for the related topic. 

This operation returns the [Topic](topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) associated with the [ContentFilteredTopic](#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic). That is, the [Topic](topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) specified when the [ContentFilteredTopic](#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic) was created. 

### const std::string &get_filter_expression() const

Get the filter expression. 

This operation returns filter expression associated with this [ContentFilteredTopic](#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic). It will return the `filter_expression` specified on the last successful call to `set_expression` or, if that method is never called, the expression specified when the [ContentFilteredTopic](#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic) was created.

* **Returns:**
  the `filter_expression`. 

### ReturnCode_t get_expression_parameters(std::vector<std::string> &expression_parameters) const

Get the expression parameters. 

This operation returns expression parameters associated with this [ContentFilteredTopic](#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic). These will be the `expression_parameters` specified on the last successful call to `set_expression` or `set_expression_parameters`. If those methods have never been called, the expression parameters specified when the [ContentFilteredTopic](#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic) was created will be returned.

* **Parameters:**
  **expression_parameters** – **[out]** The expression parameters currently associated with the [ContentFilteredTopic](#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic).
* **Returns:**
  RETCODE_OK 

### ReturnCode_t set_expression_parameters(const std::vector<std::string> &expression_parameters)

Set the expression parameters. 

This operation changes expression parameters associated with this [ContentFilteredTopic](#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic).

* **Parameters:**
  **expression_parameters** – **[in]** The expression parameters to set.
* **Returns:**
  RETCODE_OK if the expression parameters where correctly updated. 
* **Returns:**
  RETCODE_BAD_PARAMETER if the expression parameters do not match with the current `filter_expression`. 

### ReturnCode_t set_filter_expression(const std::string &filter_expression, const std::vector<std::string> &expression_parameters)

Set the filter expression and the expression parameters. 

This operation changes the filter expression and the expression parameters associated with this [ContentFilteredTopic](#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic).

* **Parameters:**
  * **filter_expression** – **[in]** The filter expression to set. 
  * **expression_parameters** – **[in]** The expression parameters to set.
* **Returns:**
  RETCODE_OK if the expression and parameters where correctly updated. 
* **Returns:**
  RETCODE_BAD_PARAMETER if `filter_expression` is not valid for this [ContentFilteredTopic](#classeprosima_1_1fastdds_1_1dds_1_1ContentFilteredTopic). 
* **Returns:**
  RETCODE_BAD_PARAMETER if the expression parameters do not match with the `filter_expression`. 

### virtual [DomainParticipant](../domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) \*get_participant() const override

Getter for the [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant). 

* **Returns:**
  [DomainParticipant](../domain/domainparticipant.md#classeprosima_1_1fastdds_1_1dds_1_1DomainParticipant) pointer 

### FASTDDS_SQLFILTER_NAME eprosima::fastdds::dds::sqlfilter_name
