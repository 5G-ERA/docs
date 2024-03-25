# Gateway

The `Middleware` system includes diverse microservices. When a `robot/user` has to consume multiple microservices, setting up distinct endpoints for each microservice and managing them separately can be challenging. A solution for handling such tasks is placing a gateway in front of the microservices. In this way, the `robot/user` can communicate with the various microservices by using a single entry-point.

## What technologies have been used and for what purposes

The gateway for the `5G-ERA Middleware` has been implemented using the `Ocelot` and `YARP` reverse proxy server. `Ocelot` is an open-source application, which is lightweight, fast, scalable, cross-platform, and most importantly it was specifically designed for `.NET Core` Microservices Architecture. `Ocelot` is responsible for handling the incoming requests, by routing the traffic to the desired endpoints, retrieving the information from the backend, and relaying it back to the `robots/users`. On the other hand, `YARP` is as well an open source application built on top of the `.NET` environment, being designed to be easily customizable and tweaked. `YARP` will accommodate the functionalities for dynamic gateway configuration and websocket for robotic communication purposes.

# Gateway functionalities

While both technologies serve the common purpose for routing the incoming requests, their distinct functionalities are described below.

## Ocelot authentication/authorization and RBAC

The `Ocelot` also fulfils the role of an `Identity Service`, through a `REST API` implementation. The user credentials are stored in a safe manner using a `HASH + SALT` model, following current best practices and using cryptographically strong libraries from `.NET` environment. The passwords are SALT-ed and hashed using the `PBKDF2` algorithm with `HMACSHA256` hashing. Authentication is achieved by reconstructing the salted hash from the credentials provided at log-in and comparing the result with the salted hash stored in the system at registration. 
Furthermore, the security of the `Middleware` systems is enhanced using the `JWT Bearer Token` standard and `Role Based Access Control` (`RBAC`) model. More precisely when the `robots/users` are successfully authenticated they will receive a `JWT Token`. In terms of authorization, the `robots/users` will have to pass the generated token along with the request in order to be able to perform operations on the data, through the endpoints that are implemented in the `Middleware` system. Finally, the security of the system is boosted with the implementation of the `RBAC` model that will provide restricted access to the `Middleware` system based on the role of the user.

## YARP dynamic gateway configuration and WebSockets

The `Middleware` system offers the possibility to dynamically configure the `Gateway` in order to route the traffic through a websocket to a specific `Network Application` using the `YARP` reverse proxy functionalities. This allows enabling `RBAC` to the `Network Applications` and expose everything behind the `Gateway` using `SSL` termination. Moreover, this will also remove the need to enable the authentication on the `Network Application` level.
The dynamic `Gateway` configuration is accomplished through standard `REST` requests and `WebSockets`, in fact, the whole mechanism is triggered through `RabbitMQ` queueing system for both creating the new route and deleting the Route once the `Network Application` has finished conducting the task.
The messages to open or close the route are sent from the Orchestrator after the desired `Network Application` is deployed or terminated.