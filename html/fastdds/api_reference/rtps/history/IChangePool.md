<a id="ichangepool"></a>

# IChangePool

### class IChangePool

An interface for classes responsible of cache changes allocation management. 

### Public Functions

### virtual bool reserve_cache([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*&cache_change) = 0

Get a new cache change from the pool. 

* **Parameters:**
  **cache_change** – **[out]** Pointer to the new cache change.
* **Returns:**
  whether the operation succeeded or not
* **Pre:**
  `cache_change` is `nullptr`
* **Post:**
  * `cache_change` is not nullptr 
  * `*cache_change` equals `CacheChange_t()` except for the contents of `serializedPayload`

### virtual bool release_cache([CacheChange_t](../common/CacheChange/CacheChange_t.md#_CPPv4N8eprosima7fastdds4rtps13CacheChange_tE) \*cache_change) = 0

Return a cache change to the pool. 

* **Parameters:**
  **cache_change** – **[in]** Pointer to the cache change to release.
* **Returns:**
  whether the operation succeeded or not
* **Pre:**
  * `cache_change` is not `nullptr`
  * `cache_change` points to a cache change obtained from a call to `this->reserve_cache`
