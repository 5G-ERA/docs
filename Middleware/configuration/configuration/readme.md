# Orchestrator Configuration

Last time revised: 01/02/2023

## Configuration options

The Orchestrator supports a configuration option that uses the AWS Secrets Manager. The examples will show the example configuration methods to configure the secrets manager to retrieve secrets automatically. Alternatively, when not using AWS Cloud and deploying everything on-premises, the manual configuration is required.

```json
{
"CustomLogger" : {
    "LoggerName": "",
    "Url": "",
    "User": "",
    "Password": ""
  }
}
```
The full configuration template list is in [config.json](config.json) file. Use it as a reference to which environment variables should be specified.

### Secrets Manager

Having in mind the above-presented piece of configuration showcases the logger configuration.

For the configuration to be correctly read by the Secrets Manager, the Secrets have to be formatted with the following scheme:

```
Middleware-{ConfigSection}__{ConfigProperty}
```

The first part `Middleware-` represents the name of the system in the secrets Manager so the secrets can be easily filtered. The next parts are responsible for specifying the config sections and specific properties. For example, to autocomplete the `User` property from the config above, the secret has to be named in the following way:

```
Middleware-CustomLogger__LoggerName
Middleware-CustomLogger__Url
Middleware-CustomLogger__User
Middleware-CustomLogger__Password
```

The values in the Secrets manager should be stored as plaintext and contain value only. 

## Manual configuration
Manual configuration of the Middleware is performed by specifying the environment variables in the deployment file, [orchestrator.yaml](https://github.com/5G-ERA/middleware/blob/main/k8s/orchestrator/orchestrator.yaml). In the `env` section of the file, we fill in the environment variables values. 

The schema of the variables is very similar to the one used with AWS Secret Manager:
```
{ConfigSection}__{ConfigProperty}
```
As you can see, it is almost identical, but without `Middleware` prefix.

## Required configuration sections

When configuring Middleware, some configuration sections are compulsory, some not.

### Always compulsory
Always compulsory configuration section is `Middleware` section. In this section we have to fill in the following properties:

```json
{
  "Middleware": {
    "Organization": "",
    "InstanceName": "",
    "InstanceType": "",
    "Address": ""
  }
}
```
* Organization: Name of the group in which current Middleware will operate in
* Instance Name: Unique name of the Middleware
* Instance Type: type of the Middleware deployment (Cloud or Edge)
* Address: DNS name under which the Middleware will be accessed

### Deployment dependant sections
The following configuration sections must be filled in unless, you are deploying Middleware with the AWS Secrets Manager and specify `AWS_ACCESS_KEY_ID` and `AWS_SECRET_ACCESS_KEY` environment variables.

* CustomLogger
* InfluxDB
* RabbitMQ
* Redis

### Always optional sections

The always optional sections of the Middleware configuration can be omitted and default values will be used. If you want to specify how Middleware is exposed, what is the default user and password, or you want to use Slice Switchover capabilities, modify these sections:

* GatewayConfig
* User
* Slice

### Logging

The Middleware currently supports two log aggregation systems. It allows sending logs to either `Elasticsearch` or `Grafana Loki`.

The `LoggerName` field is used to decide which of them will be used.

The `LoggerName` property allows for two entries:
* **Elasticsearch**
* **Loki**
