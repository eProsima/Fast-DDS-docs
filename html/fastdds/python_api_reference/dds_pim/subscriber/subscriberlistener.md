<a id="python-api-pim-subscriberlistener"></a>

<a id="subscriberlistener"></a>

# SubscriberListener

### *class* fastdds.SubscriberListener

Class SubscriberListener, it should be used by the end user to implement specific callbacks to certain actions.
It also inherits all DataReaderListener callbacks.

#### on_data_on_readers(sub)

Virtual function to be implemented by the user containing the actions to be performed when a new
Data Message is available on any reader.

* **Parameters:**
  **sub** ([`Subscriber`](subscriber_class.md#fastdds.Subscriber)) – Subscriber

#### *property* thisown

The membership flag
