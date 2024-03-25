# Central API

Central API is the key connection point between the available Middlewares.

The Central API plays the routing role between the deployed Middleware instances. It is responsible for making other Middleware locations available to each other and informing them about their locations. 

When the Middleware starts it has to be onboarded to the Middleware database to enable the correct registration. 

When the Middleware instance starts up, it sends the request to the Central API to register itself and set online status. 

## Deployment

The deployment scripts for the Central API are located in the [k8s/central-api](../../../k8s/central-api/) folder.