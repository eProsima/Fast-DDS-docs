<a id="api-pim-writerresourcelimitsqos"></a>

<a id="writerresourcelimitsqos"></a>

# WriterResourceLimitsQos

### class WriterResourceLimitsQos

Qos Policy to configure the limit of the writer resources. 

### Public Functions

### inline WriterResourceLimitsQos()

Constructor. 

### virtual ~WriterResourceLimitsQos() = default

Destructor. 

### Public Members

### fastdds::ResourceLimitedContainerConfig matched_subscriber_allocation

Matched subscribers allocation limits. 

### fastdds::ResourceLimitedContainerConfig reader_filters_allocation

Reader filters allocation limits.
