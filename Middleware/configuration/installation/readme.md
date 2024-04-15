# Installation

Middleware installation process includes a few steps that are required 
to provision the environment ready to run the Middleware.

The steps include infrastructure deployment, supporting services deployment (required to be deployed only once),
CentralApi deployment (also deployed only once) and finally the Middleware. 
## Infrastructure

Infrastructure required to run the Middleware on depends on the environment we are deploying it in.
There are different requirements to deploy Middleware in Cloud and different for local Edge deployments.

To deploy in Cloud, the special Kubernetes infrastructure is needed, while for Edge local Kubernetes cluster of chosen flavor is sufficient.

The detailed infrastructure deployment instructions are in [infrastructure deployment document](./infrastructure/readme.md).

## Supporting Services

Supporting services are responsible for work of the Middleware system, they are Redis for Graph and NoSQL database, 
RabbitMQ messaging system, Grafana loki Centralized logging system and InfluxDB for time series robot and NetApp insights.

To deploy them, the infrastructure is required.
## CentralApi

## Middleware