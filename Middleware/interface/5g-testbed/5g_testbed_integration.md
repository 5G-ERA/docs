# 5G Testbed integration

Middleware is designed to be able to enable vertical applications the connection to network applications. 

For the integration, Middleware exposes the API endpoint allowing the external `SliceManagerAPI` to update the Middleware about the latest changes to the available network Slices. The `SliceManagerAPI` is a testbed-controlled API responsible for the customized association of the Slices with the `Imsis'`. Due to the nature of the testbeds, Middleware does not provide its own `SliceManagerAPI`, for the integration it has to be provided by the testbed provider.

During the planning Middleware expects the `SliceManager` to prepare an endpoint to enable the association of the specific `Imsi` (SIM Card Id) with the specified network Slice.


## Receiving information from `SliceManager`

Middleware does not have direct access to the information from the testbed. The external API is needed to detect the changes to the Slices configuration. When the change is detected, `SliceManager` is expected to notify the Middleware about the changes with the call containing information about all available Slices.

The Middleware exposes the endpoint to receive an update on the available slices. The endpoint is exposed under `<Middleware-IP>/data/slice` path. 
With the `POST` method, the Slices available for the current location can be updated. When the request is received and validated, the Middleware proceeds with the removal of all existing Slices associated with the current Middleware instance, and saving the newly passed Slices configurations. 

The endpoint expects the following `BODY` of the method:

```json
{    
	"slices": [
	{
		"sliceId": "Era-5G-Rome-EMBB1",
		"site": "Rome",
		"expDataRateUL": 30,
		"expDataRateDL": 300,
		"userDensity": 10,
		"userSpeed": 80,
		"trafficType": "TCP",
		"imsi": ["imsi1", "imsi2", "imsi3"]
	},	
	{
		"sliceId": "Era-5G-Rome-URLLC1",
		"site": "Rome",
		"latency": 5,
		"jitter": 1,
		"expDataRateUL": 15,
		"expDataRateDL": 50,
		"trafficType": "TCP",
		"imsi": ["imsi1", "imsi2", "imsi3"]
	}	
	]
}
```

The endpoint allows passing the configuration for both `EMBB` and `URLLC` network Slices. Both slices' configurations have a different set of properties. 

When currently available Slices are correctly updated, Middleware returns `201` HTTP status code. When the validation fails, the endpoint returns `400`. 


## Association of Imsi with Slice

As part of the resource planning process, Middleware selects the best possible location for the Network Application. This can include the requests for network Slices. After the plan is created and the deployment started, the Middleware will want to connect the specified vertical application with the specified available Slice. For this Middleware expects the `SliceManager` to expose an API `POST` endpoint to send the configuration of the Slice and `imsi`.

The endpoint the Middleware will call has path `<SliceManager-IP>/testbed/sliceUE/attach`. The endpoint has to accept the following method `BODY`:

```json
{
    "imsi": "1234kjh6543l5jh",
    "sliceId" : "Era-5G-Rome-URLLC1",
    "usedDataRateUl": 10,
    "usedDataRateDl": 10
}
```

## On-demand retrieval of the Slice Information by Middleware

When configured to utilize `SliceManager` capabilities, the Middleware can request the latest information about the available Slices. This request is used to get the latest information right after the Middleware startup and other necessary situations. 

The Middleware will call `SliceManager` on the following endpoint `<SliceManager-IP>/testbed/sliceUE` using the `GET` method. The Middleware expects the `GET` method to return the data in the same format as the data send to the Middleware's `<Middleware-IP>/data/slice` endpoint, which is in the following format:

```json
{    
	"slices": [
	{
		"sliceId": "Era-5G-Rome-EMBB1",
		"site": "Rome",
		"expDataRateUL": 30,
		"expDataRateDL": 300,
		"userDensity": 10,
		"userSpeed": 80,
		"trafficType": "TCP",
		"imsi": ["imsi1", "imsi2", "imsi3"]
	},	
	{
		"sliceId": "Era-5G-Rome-URLLC1",
		"site": "Rome",
		"latency": 5,
		"jitter": 1,
		"expDataRateUL": 15,
		"expDataRateDL": 50,
		"trafficType": "TCP",
		"imsi": ["imsi1", "imsi2", "imsi3"]
	}	
	]
}
```
The expected return status code is `200`. 