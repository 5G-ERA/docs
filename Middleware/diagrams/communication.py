from diagrams import Cluster, Diagram
from diagrams.aws.compute import EC2, EKS, ECS
from diagrams.aws.network import NLB
from diagrams.custom import Custom
from diagrams.k8s.compute import Pod
from diagrams.onprem.inmemory import Redis
from diagrams.onprem.network import Nginx
from diagrams.onprem.queue import RabbitMQ

with (Diagram("", direction="LR", show=False, filename="communication")):
    robot = Custom("Robot", "../img/summit-xl.jpg")
    with Cluster("AWS") as cloud:
        middleware_cloud = EKS("Middleware")
        netapp = Pod("SLAM")
        middleware_cloud >> netapp

    with Cluster("Edge") as edge:
        middleware_edge = Custom("Middleware", "../img/k8s.png")
        netapps = Pod("SLAM")
        middleware_edge >> netapps

    with Cluster("CentralApi") as central_api:
        api = EC2("CentralApi")
    with Cluster("Infrastructure") as infra:
        rabbit =RabbitMQ("RabbitMQ")
        redis = Redis("Redis")

    robot >> [middleware_cloud, middleware_edge, api]
    middleware_cloud >> [rabbit, redis]
    middleware_edge >> [rabbit, redis]
    api >> [rabbit, redis]

