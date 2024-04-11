from diagrams import Diagram, Cluster
from diagrams.aws.network import NLB
from diagrams.aws.compute import EC2

from diagrams.custom import Custom
from diagrams.k8s.compute import Deploy
from diagrams.k8s.network import Ingress
from diagrams.onprem.network import Nginx
from diagrams.onprem.database import InfluxDB
from diagrams.onprem.inmemory import Redis
from diagrams.onprem.monitoring import Grafana
from diagrams.onprem.queue import RabbitMQ



with Diagram("Architecture", direction="TB", show=False, filename="crop-architecture"):
    robots = Custom("Robot", "../img/summit-xl.jpg")
    #admin = User("Admin")
    #nlb = NLB("NLB")
    with Cluster("AWS Cluster", direction="TB") as infra:
        with Cluster("Services") as services:
            redis = Redis("Redis")
            rabbitmq = RabbitMQ("RabbitMQ")
            grafana = Grafana("Grafana")
            influx = InfluxDB("InfluxDB")
        with Cluster("Middleware", direction="TB") as middleware:
            gateway = Nginx("Gateway")
            mw = EC2("Middleware services")
            orchestrator = EC2("Orchestrator")
            gateway >> mw >> orchestrator
            mw >> [redis, influx, rabbitmq]
        with Cluster("Network Applications") as net_apps:
            netapps = [EC2("Object Detection"), EC2("SLAM"), EC2("Teleoperation")]

            pass
    gateway >> netapps
    #robots >> nlb >> [gateway]
    robots >> [gateway]
    #orchestrator >> [netapp, netapp2]