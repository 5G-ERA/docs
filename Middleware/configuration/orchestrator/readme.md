# Orchestrator Configuration

Last time revised: 01/02/2023

## Configuration options

The Orchestrator supports a configuration option that uses the AWS Secrets Manager. The examples will show the example configuration methods to configure the secrets manager to retrieve secrets automatically.

```json
"CustomLogger" : {
    "LoggerName": "",
    "Url": "",
    "User": "",
    "Password": ""
  }
```

### Secrets Manager

Having in mind the above-presented piece of configuration showcases the logger configuration.

For the configuration to be correctly read by the Secrets Manager the Secrets have to be formatted with the following scheme:

```
Middleware-{ConfigSection}__{ConfigProperty}
```

The first part `Middleware-` represents the name of the system in the secrets Manager so the secrets can be easily filtered. The next parts are responsible for specifying the config sections and specific properties. For example, to autocomplete the `User` property from the config above, the secret has to be named in the following way:

```
Middleware-CustomLogger__User
```

The values in the Secrets manager should be stored as plaintext and contain value only. 
### Logging

The Middleware currently supports two log aggregation systems. It allows sending logs to either `Elasticsearch` or `Grafana Loki`.

The `LoggerName` field is used to decide which of them will be used.

The `LoggerName` property allows for two entries:
* **Elasticsearch**
* **Loki**