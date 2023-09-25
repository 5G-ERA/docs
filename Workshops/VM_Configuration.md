# Microk8s configuration on the VM

This tutorial assumes that you have Ubuntu installed in either Desktop or a Server version.

### Docker installation

The docker is required to build the image during the second stage of the workshop and the first part of the workshop. 
Start with updating your package repository and download the packages required.

```shell
$ sudo apt-get update

$ sudo apt-get install \
    ca-certificates \
    curl \
    gnupg \
    lsb-release
```

Configure the additional repository for docker.

```shell
 $ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

 $ echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

```

Update the packages repository and install Docker required packages.

```shell
$ sudo apt-get update

$ sudo apt-get install docker-ce docker-ce-cli containerd.io
```

Post installation steps, are not required, but are helpful if you don't want to type `sudo` with every docker command that you run.
These commands create the new group `docker` and add the current user to this group, so it has the permissions to execute `docker` commands. It is possible that after the installation `docker` group is already created. It is not a problem, follow to the next command.

After the user is added to the group, we immediately apply the changes so there is no need to reboot the VM.
```shell
$ sudo groupadd docker

$ sudo usermod -aG docker $USER

$ su - $USER
```
With this step finished, the `docker` is properly configured. 


### Microk8s installation

The next step is to configure the minimal Kubernetes client to conduct the workshop experiments on. 

The minimal Kubernetes installation used will be Microk8s. 

Install the Microk8s with the following command. Make sure that the `snap` is installed on your machine. If you use the Ubuntu distribution, it should come pre-installed. If not, follow the instructions from the [official source](https://snapcraft.io/docs/installing-snapd).

```shell
$ sudo snap install microk8s --classic
```

With the Microk8s installed, we will repeat the steps that we made for docker installation. To ease the process of running commands without `sudo`.
Add the current user to the `microk8s` group, and give the ownership of the `~./kube` folder to the current user. In the end execute the login for the user to apply immediately the changes in the groups. 

```shell
$ sudo usermod -a -G microk8s $USER

$ sudo chown -f -R $USER ~/.kube

$ su - $USER
```

Now initialize the installation of the Microk8s. 

```shell
$ microk8s status --wait-ready
```
After the Microk8s is initialized, add an alias, so the command `kubectl` can be used to interact with the Microk8s.
```shell
$ alias kubectl='microk8s kubectl'
```

In the end the addons in the Microk8s have to be enabled to ensure the correct work of the ROS2 applications.
```shell
$ microk8s enable dns multus
```
After the addons have been added the process of configuration is complete.