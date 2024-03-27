# Route configuration for Network Applications
The CROP Middleware is responsible for creating the routes between the robots and their respective Network Applications during the deployment process. CROP Middleware creates a new entry in the gateway specification file which contains:
- the protocol: `http` or `https`,
- the domain: (containing the network application name and middleware address) `teleoperation.middlewareaddress`
- port: `80`, 
- example: `http://teleoperation.middleware-address:80`



## Cofigure DNS Server
