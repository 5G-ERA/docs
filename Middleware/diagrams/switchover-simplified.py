from diagrams import Cluster, Diagram
from diagrams.aws.compute import EC2, EKS
from diagrams.custom import Custom
from diagrams.k8s.compute import Pod

with Diagram("", direction="RL", show=False, filename="switchover-simplified"):
    robot = Custom("Robot", "../img/summit-xl.jpg")
    with Cluster("Edge") as edge:
        middleware_edge = Custom("Middleware", "../img/k8s.png")
        net_app = EC2("SLAM")
    with Cluster("Cloud") as cloud:
        middleware_cloud = Custom("Middleware", "../img/k8s.png")
        net_app2 = EC2("SLAM")

    robot >> middleware_edge >> net_app
    robot >> middleware_cloud >> net_app2
    net_app >> middleware_cloud