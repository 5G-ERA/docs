from diagrams import Cluster, Diagram
from diagrams.custom import Custom
from diagrams.k8s.compute import Deploy
from diagrams.k8s.infra import Node
from diagrams.onprem.inmemory import Redis

with Diagram("", direction="BT", show=False, filename="gateway"):
    robot = Custom("Robot", "../img/summit-xl.jpg")
    with Cluster("middleware") as middleware:
        gateway = Deploy("CentralApi")
        orchestrator = Deploy("orchestrator")
        task_planner = Deploy("TaskPlanner")
        redis_interface = Deploy("Redis Interface")
        net_app = Deploy("Network Application")

    robot >> gateway
    gateway >> [orchestrator, task_planner, redis_interface, net_app]