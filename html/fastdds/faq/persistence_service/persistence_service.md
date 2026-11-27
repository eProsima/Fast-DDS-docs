<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="freq-persistence-service-questions"></a>

<a id="persistence-service-frequently-asked-questions"></a>

# PERSISTENCE SERVICE Frequently Asked Questions

**What is persistence in eProsima Fast DDS?**

<br/> Persistence is a mechanism that allows recovering a previous state on starting the DDS.
This is done by configuring the DataWriter’s history to be stored in a persistent database, so that the DataWriter can load its history from it on creation.
Furthermore, DataReaders can be configured to store the last notified change in the database, so that they can recover their state on creation.
For more information, refer to [Persistence Service](../../persistence/persistence.md#persistence-service).

---

**Why is it useful?**

<br/> It adds robustness to applications in case of unexpected shutdowns.
For further information, refer to [Persistence Service](../../persistence/persistence.md#persistence-service).

---

**How is the persistence database managed?**

<br/> A persistence plugin must be configured for managing the database using the property `dds.persistence.plugin`.
For more information, refer to [Configuration](../../persistence/persistence.md#persistence-service-conf).

---

**What type of database does the persistence plugin use?**

<br/> It uses SQLite3 API.
For more information, refer to [PERSISTENCE:SQLITE3 built-in plugin](../../persistence/persistence.md#persistence-sqlite3-builtin-plugin).

<br/>
