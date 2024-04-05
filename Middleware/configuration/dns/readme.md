# DNS

In the local Edge envionments that are not assigned a public domain name, there is a need to configure local DNS setting that will enable accessing the Middleware instance using fully qualified DNS names.

The DNS names allow the Middleware Gateway to route the traffic to the approperiate NetApps. For example `ros-object-detection.middleware.net` will route the traffic to the `ros-object-detection` NetApp that is exposed by the Middleware under `middleware.net` address.

## Configuration
To configure the CoreDNS configure the files attached in the current directory.
To specify the desired domain name to be used, `Corefile` and `middleware.db` files must be modified. Change `middleware.io` domain to the desired address in these files. The names must be mapped to the IP address of the machine where the Middleware is running.

## Deployment
There are two ways of starting the CoreDNS container. 
The first one uses `docker` command:
```shell
docker run -d --name coredns --restart=always -v ./:/root/ -p 53:53/udp coredns/coredns -conf /root/Corefile
```

The second one uses `docker compose` file:

```shell
docker compose up -d
```

## Router configuration
After the container has started we have to configure the name resolution across our network. To achieve this we need to propagate DNS server to all the machines. The easiest way to achieve it is to configure our router, to point local network to the new server. 

Go to `LAN -> DHCP -> DNS` section in your router configuration and in the DNS fields, type the address of the machine that hosts CoreDNS. 

Save the changes in the router and try to access your desired address from the web browser on one of the machines. If you see Middleware banner appear, it means that you have successfully configured your DNS server. 

When the address is not resolved, try restarting the router
## Possible issues

There are a few possible issues that can occur during installation and configuration of the CoreDNS.

### Port 53 is already occupied

On Linux systems there is a chance when starting up the CoreDNS container that the port `53` is already in use by other service. It is very possible that the `systemd-resolved` service is utilising this port. 

To enable this we will need to stop and disable the service:

```bash
systemctl disable systemd-resolved.service
systemctl stop systemd-resolved
```

After changing this setting the port `53` will be available but you will not have DNS working on this machine.

To fix this, edit `/etc/resolved.conf` file and change the address in  `nameserver 127.0.0.53` line to the IP address of your router.

After you configure DNS on your router. The machine that hosts CoreDNS should now have an access to the internet using DNS services and should resolve our new addresses.