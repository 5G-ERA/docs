# TLS Configuration

The configuration for TLS has been implemented with `Cert Manager` and `Nginx Ingress Controller`.

## Setup Cert-manager
### Installing cert-manager with helm
### 1. Adding helm repository:

```
helm repo add jetstack https://charts.jetstack.io
```

### 2. Updating helm repository:

```
helm repo update
```

### 3. Installing Custom Resource Definitions CRD:

```
kubectl apply -f https://github.com/cert-manager/cert-manager/releases/download/v1.13.3/cert-manager.crds.yaml
```

### 4. Installing Cert Manager:

```
helm install cert-manager jetstack/cert-manager --namespace cert-manager --create-namespace --version v1.13.2
```


## Setup Nginx Ingress Controller
### Installing nginx with helm
### 1. Adding helm repository:

```
helm repo add ingress-nginx https://kubernetes.github.io/ingress-nginx
```

### 2. Updating helm repository:

```
helm repo update
```

### 3. Modify values.yaml file:
The nginx chart contains a `values.yaml` file, this file requires a few modifications in the `ipFamilyPolicy` section, in order to function with the current Middleware architecture. The initial configuration for the nginx controller service comes with `type: LoadBalancer`. Since the Middleware is already exposed through a Network Load Balancer, this has to be adjusted to `type: NodePort`. Also in the Network Load Balancer that serves the Middleware system the listeners and target groups are assigned to: `http: 80:31000` and `https: 443:31011`, these properties also have to be adjusted to match the same values. The below code depicts the required modifications:   

```
    ipFamilyPolicy: "SingleStack"
    ipFamilies:
      - IPv4
    ports:
      http: 80
      https: 443
    targetPorts:
      http: http
      https: https
    type: NodePort
    nodePorts:
      http: 31000
      https: 31011
      tcp: {}
      udp: {}
    external:
      enabled: true
    internal:
```

### 4. Installing Nginx Ingress Controller:
To apply the configuration that was modified in the previous step, when installing the nginx controller, specify the `-f values.yaml` in the installation command:

```
helm install ingress-nginx ingress-nginx/ingress-nginx --namespace ingress --create-namespace --set controller.ingressClassResource.name=nginx -f values.yaml
```

### 5. Check the service configuration:

```
kubectl -n ingress get svc
```

The desired result should look like below:
```
NAME                               TYPE       CLUSTER-IP       EXTERNAL-IP   PORT(S)                      AGE
service/ingress-nginx-controller   NodePort   172.20.190.113   <none>        80:31000/TCP,443:31011/TCP   11d
```

## Domain registration
The domain for the Middleware system has been registered under Amazon Route 53, check following link:
```
https://docs.aws.amazon.com/Route53/latest/DeveloperGuide/domain-register.html
```
The TLS certificate authority used is Let's Encrypt. For other options for domain providers check the following list of compatibility with Let's Encrypt, under the following link:

```
https://community.letsencrypt.org/t/dns-providers-who-easily-integrate-with-lets-encrypt-dns-validation/86438
```



## Setup Ingress for Gateway and Central-API
First we setup the ingress for both services we want to access through the nginx controller without configuring the TLS yet.

### 1. Ingress for the Gateway:
The host name for the gateway service is `crop.5gera.net`, however, in the later step when we will have to configure the cert-manager, we want to provide a wildcard certificate that will be serving for other subdomains as well, hence we will use the host `*.5gera.net`, with this host, cert-manager will generate a single tls certificate that can be used for other subdomains that will be registered under `5gera.net`.\
Create the ingress for the Gateway in a `ingress-gateway.yaml` file as below, and leave the commented out code as it is for now:

```
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: ingress-gateway
  namespace: middleware
  #This section will be uncommented after the cluster-issuer in the cert-manager is configured
  #annotations:
  #  cert-manager.io/cluster-issuer: "letsencrypt-prod"
spec:
  defaultBackend:
    service:
      name: gateway
      port:
        number: 80
  ingressClassName: nginx
  rules:
    - host: "*.5gera.net"
    - http:
        paths:
          - pathType: Prefix
            backend:
              service:
                name: gateway
                port:
                  number: 80
            path: /
  # This section will be uncommented after the cluster-issuer in the cert-manager is configured
  #tls:
  #  - hosts:
  #    - "*.5gera.net"
  #    secretName: tls-secret
```

Apply the yaml configuration file with: 

```
kubectl apply -f ingress-gateway.yaml
```

Check the ingress configuration with:

```
kubectl -n middleware get ingress
```
The desired result should look like below:
```
NAME              CLASS   HOSTS         ADDRESS          PORTS     AGE
ingress-gateway   nginx   *.5gera.net   172.20.190.113   80        11d
```

### 2. Ingress for the Central-API:
For the Central-API we will use the specific host which is `central-api.5gera.net`.\
Create the ingress for the Central-API in a `ingress-central-api.yaml` file as below:

```
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: ingress-central-api
  namespace: central-api
spec:
  defaultBackend:
    service:
      name: central-api
      port:
        number: 80
  ingressClassName: nginx
  rules:
    - host: central-api.5gera.net
      http:
        paths:
          - pathType: Prefix
            backend:
              service:
                name: central-api
                port:
                  number: 80
            path: /?(.*)
```
Apply the yaml configuration file with: 

```
kubectl apply -f ingress-central-api.yaml
```

Check the ingress configuration with:

```
kubectl -n central-api get ingress
```
The desired result should look like below:
```
NAME                  CLASS   HOSTS                   ADDRESS          PORTS   AGE
ingress-central-api   nginx   central-api.5gera.net   172.20.190.113   80      11d
```
## Setup Cert Manager Issuer to obtain the TLS certificate

There are multiple ways in which one can obtain the TLS certificate through the cert-manager issuer, for this tutorial we will create a new aws user, an access key ID, and new policy for aws route 53.

### 1. AWS User:
Follow the link below to create a new user:
```
https://docs.aws.amazon.com/IAM/latest/UserGuide/id_users_create.html
```

### 2. AWS access key ID:
Follow the link below to create a new access key ID:
```
https://docs.aws.amazon.com/IAM/latest/UserGuide/id_credentials_access-keys.html
```

### 3. AWS Policy:
Follow the link below to create a new policy:
```
https://docs.aws.amazon.com/IAM/latest/UserGuide/access_policies_create-console.html
```
Use the following json template for your new policy:
```
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Effect": "Allow",
            "Action": "route53:GetChange",
            "Resource": "arn:aws:route53:::change/*"
        },
        {
            "Effect": "Allow",
            "Action": [
                "route53:ChangeResourceRecordSets",
                "route53:ListResourceRecordSets"
            ],
            "Resource": "arn:aws:route53:::hostedzone/*"
        },
        {
            "Effect": "Allow",
            "Action": "route53:ListHostedZonesByName",
            "Resource": "*"
        }
    ]
}
```
### 4. Attach policy to the user
Follow the link below to attach the policy to the user:
```
https://docs.aws.amazon.com/apigateway/latest/developerguide/api-gateway-create-and-attach-iam-policy.html
```

### 5. Cert Manager ClusterIssuer:
The issuer for cert-manager has been implemented as `ClusterIssuer` and had to be deployed in the same namespace as the `cert-manager`. When obtaining the TLS certificate, Lets Encrypt has to check the ownership of the domain, cert-manager will create a challenge based on the configuration that is provided in the issuer to fulfill the ownership check with Lets Encrypt.\
Create the ClusterIssuer in a `cluster-issuer.yaml` as below, using the credentials created in the previous steps:
```
apiVersion: cert-manager.io/v1
kind: ClusterIssuer
metadata:
  name: letsencrypt-prod
  namespace: cert-manager
spec:
  acme:
    email: email@example.uk
    privateKeySecretRef:
      name: letsencrypt-prod
    server: https://acme-v02.api.letsencrypt.org/directory
    solvers:
    # this solver handles example.org challenges
    # and uses explicit credentials
    - selector: {}
      dns01:
        route53:
          region: eu-west-2
          # The AWS access key ID can be specified using the literal accessKeyID parameter
          # or retrieved from a secret using the accessKeyIDSecretRef
          # If using accessKeyID, omit the accessKeyIDSecretRef parameter and vice-versa
          accessKeyID: AKIAAWSEXAMPLEACCKEY
          #accessKeyIDSecretRef:
          #  name: prod-route53-credentials-secret
          #  key: access-key-id
          secretAccessKeySecretRef:
            name: certmanager-route53
            key: secret-access-key
```
Apply the yaml configuration file with: 

```
kubectl apply -f cluster-issuer.yaml
```

Check the ClusterIssuer configuration with:

```
kubectl -n cert-manager get ClusterIssuer
```
The desired result should look like below:
```
NAME               READY   AGE
letsencrypt-prod   True    11d
```


## Securing the Ingress with TLS certificate
The last step in the process is to uncomment the code in the ingress for the Gateway, and re apply the file with the new configuration for TLS certificate. Modify the `ingress-gateway.yaml` as below:

```
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: ingress-gateway
  namespace: middleware
  #This section will be uncommented after the cluster-issuer in the cert-manager is configured
  annotations:
    cert-manager.io/cluster-issuer: "letsencrypt-prod"
spec:
  defaultBackend:
    service:
      name: gateway
      port:
        number: 80
  ingressClassName: nginx
  rules:
    - host: "*.5gera.net"
    - http:
        paths:
          - pathType: Prefix
            backend:
              service:
                name: gateway
                port:
                  number: 80
            path: /
  # This section will be uncommented after the cluster-issuer in the cert-manager is configured
  tls:
    - hosts:
      - "*.5gera.net"
      secretName: tls-secret
```
Apply the yaml configuration file with: 

```
kubectl apply -f ingress-gateway.yaml
```

Check the ingress configuration with:

```
kubectl -n middleware get ingress
```
The desired result should look like below, notice the port 443 in the new ingress:
```
NAME              CLASS   HOSTS         ADDRESS          PORTS     AGE
ingress-gateway   nginx   *.5gera.net   172.20.190.113   80, 443   11d1h
```
The ingress for central-api can remain with the same configuration, the nginx-ingress-controller will automatically forward the traffic that is coming on port 80/http to port 443/https. The TLS configuration is also not required in the ingress for the central-api, as explained above the wildcard certificate will serve for all other subdomains registered under the `5gera.net` domain, and this option has already been enabled when applying the new `ingress-gateway.yaml`.

## Troubleshooting
In the last step of this tutorial, where the ingress for the Gateway has been re-applied, in the last line of the ingress we have added the `secretName: tls-secret`. Cert Manager will place the certificate that was retrieved from Lets Encrypt inside this variable `tls-secret`. The Nginx Ingress Controller should pick up this `tls-secret` and place it in the nginx-ingress-controller pod. However, it might happen that the Nginx Ingress Controller will pick up the configuration change, will redeploy the pod and try to reload the certificate as configured, but fail.\
To check for the right configuration, execute in the nginx pod and look for the certificate that was loaded in the `nginx.conf`, see below:
```
rhadoo@rhadoo:~$ k -n ingress get pods
NAME                                        READY   STATUS    RESTARTS   AGE
ingress-nginx-controller-675c77997b-zr9z6   1/1     Running   0          11d22h
rhadoo@rhadoo:~$ k -n ingress exec -it ingress-nginx-controller-675c77997b-zr9z6 /bin/bash
ingress-nginx-controller-675c77997b-zr9z6:/$ cd /etc/nginx/
ingress-nginx-controller-675c77997b-zr9z6:/etc/nginx$ grep .pem nginx.conf
        ssl_certificate     /etc/ingress-controller/ssl/middleware-tls-secret.pem;
        ssl_certificate_key /etc/ingress-controller/ssl/middleware-tls-secret.pem;
```
If the certificate has not be loaded properly and it shows as `/etc/ingress-controller/ssl/fake/certificate.pem`, you will have to manually adjust this in the nginx ingress controller `deployment.yaml` file, by adding in the containers `args:` section, in the last entry, the following line:
```
--default-ssl-certificate=middleware/tls-secret
```
See below:
```
spec:
      containers:
      - args:
        - /nginx-ingress-controller
        - --publish-service=$(POD_NAMESPACE)/ingress-nginx-controller
        - --election-id=ingress-nginx-leader
        - --controller-class=k8s.io/ingress-nginx
        - --ingress-class=nginx
        - --configmap=$(POD_NAMESPACE)/ingress-nginx-controller
        - --default-ssl-certificate=middleware/tls-secret
```
When saving the changes made in the `deployment.yaml` file, the nginx ingress controller will pick up the change and redeploy a pod that will contain the correct certificate. For more details, the following link explains how to fix the same issue:
```
https://stackoverflow.com/questions/71127151/ssl-certificate-added-but-shows-kubernetes-ingress-controller-fake-certificate
```