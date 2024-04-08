# Middleware configuration

This directory showcases the required configuration to run the Middleware and possible options that it offers when it comes to the different ways it can be run.

There are a few steps required to be completed before the use of the 5G-ERA Middleware.

## Installation
Before Middleware can be used, it needs to have all the necessary infrastructure installed. 
For the detailed instructions on how to install the required infrastructure, 
see an [Installation guide](./installation/readme.md)

## Configuration
Proper configuration of the Middleware is required before the deployment, for more details, 
see a [configuration guide](./configuration/readme.md).

## Routing and local DNS for Edge deployments
To fully use Middleware capabilities, for the local Edge deployments,
without a public IP address, the local DNS is required to resolve DNS name.
As the 5G-ERA Middleware is responsible
for creating the routes between the robots and their respective Network Applications during the deployment process.
CROP Middleware uses reverse proxy and creates a new entry in the specification file which contains:
- the domain: (containing the network application name and middleware address) `teleoperation.middlewareaddress.net`
- port: `80`

Example: `http://teleoperation.middleware-address.net:80`

This address allows the dynamic utilization of multiple network applications and constant access to them. 

To see how to configure local DNS server and use it in the local network, see the [respective guide](./dns/readme.md).