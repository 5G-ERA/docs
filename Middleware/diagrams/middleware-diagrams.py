from diagrams import Diagram, Cluster
from diagrams.aws.network import NLB

from diagrams.custom import Custom
from diagrams.k8s.compute import Deploy
from diagrams.k8s.network import Ingress
from diagrams.onprem.database import InfluxDB
from diagrams.onprem.inmemory import Redis
from diagrams.onprem.monitoring import Grafana
from diagrams.onprem.queue import RabbitMQ



with Diagram("Architecture", direction="LR", show=False, filename="crop-architecture"):
    robots = Custom("Robot", "../img/summit-xl.jpg")
    #admin = User("Admin")
    nlb = NLB("NLB")
    with Cluster("Infrastructure") as infra:
        redis = Redis("Redis")
        rabbitmq = RabbitMQ("RabbitMQ")
        grafana = Grafana("Grafana")
        influx = InfluxDB("InfluxDB")

    with Cluster("CentralApi") as central_api:
        central_api_pod = Deploy("CentralApi")
        dashboard = Deploy("Dashboard")
        central_api_pod >> redis

    with Cluster("Middleware") as middleware:
        gateway = Ingress("Gateway")
        redis_interface = Deploy("RedisInterface")
        orchestrator = Deploy("orchestrator")
        task_planner = Deploy("TaskPlanner")
        resource_planner = Deploy("ResourcePlanner")
        pods = [redis_interface, orchestrator, task_planner, resource_planner]
        gateway >> pods
        redis_interface >> redis
        orchestrator >> influx
        [orchestrator, task_planner, resource_planner] >> rabbitmq
        [orchestrator, task_planner, resource_planner] >> redis_interface
    with Cluster("Network Applications") as net_apps:
        netapp = Deploy("Network Application")
        pass
    gateway >> netapp
    robots >> nlb >> [central_api_pod, gateway, grafana, dashboard]