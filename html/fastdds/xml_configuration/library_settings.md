<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="intra-process-delivery-xml-profile"></a>

<a id="intra-process-delivery-profiles"></a>

# Intra-process delivery profiles

This section defines the XML elements available for configuring the [Intra-process delivery](../transport/intraprocess.md#intraprocess-delivery) settings
parameters in *Fast DDS*.
These elements are defined within the XML tag `<library_settings>`.

<a id="intra-process-delivery-configuration"></a>

## Intra-process delivery configuration

The Intra-process delivery configuration is performed through the XML elements listed in the following table.

| Name                      | Description                                                                                                                           | Values                                            | Default   |
|---------------------------|---------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------|-----------|
| `<intraprocess_delivery>` | Speed up communications between entities within the same process by avoiding<br/>any of the overhead involved in the transport layer. | `OFF` <br/><br/>`USER_DATA_ONLY` <br/><br/>`FULL` | `OFF`     |

**Example**

```xml
    <library_settings>
        <intraprocess_delivery>FULL</intraprocess_delivery> <!-- OFF | USER_DATA_ONLY | FULL -->
    </library_settings>
```
