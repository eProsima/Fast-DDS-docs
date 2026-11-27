<a id="python-api-pim-presentationqospolicyaccessscopekind"></a>

<a id="presentationqospolicyaccessscopekind"></a>

# PresentationQosPolicyAccessScopeKind

### fastdds.INSTANCE_PRESENTATION_QOS

Scope spans only a single instance. Indicates that changes to one instance need not be coherent nor ordered with
respect to changes to any other instance. In other words, order and coherent changes apply to each instance
separately.

### fastdds.TOPIC_PRESENTATION_QOS

Scope spans to all instances within the same DataWriter (or DataReader), but not across instances in different
DataWriter (or DataReader).

### fastdds.GROUP_PRESENTATION_QOS

Scope spans to all instances belonging to DataWriter (or DataReader) entities within the same Publisher (or Subscriber).
