# CentralApi

CentralApi is responsible for synchronization and discovery of multiple Middleware instances. 
To deploy CentralApi we need the supporting infrastructure deployed and to retrieve the addresses of the loggers and Redis.


## Deployment

### With AWS
To deploy CentralApi in the AWS public cloud, fill in the `central-api-secret.yaml` with `AWS_ACCESS_KEY_ID` and `AWS_SECRET_ACCESS_KEY`.
They are used to authenticate with AWS and retrieve credentials from the Secrets Manager.

### Without AWS
Correct environment variables must be filled in the `central-api.yaml` file, used to deploy CentralApi.
The values from them will be used to initiate the connection with the supporting services and enable correct work of the CentralApi.

To deploy CentralApi use `config.sh` script located in this directory.

```Shell
./config.sh
```
