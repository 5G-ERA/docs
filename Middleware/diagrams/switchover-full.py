from diagrams import Cluster, Diagram
from diagrams.aws.compute import EC2, EKS
from diagrams.aws.network import NLB
from diagrams.custom import Custom
from diagrams.k8s.compute import Pod
from diagrams.onprem.network import Nginx

with Diagram("", direction="TB", show=False, filename="switchover-full"):
    robot = Custom("Robot", "../img/summit-xl.jpg")
    with Cluster("AWS") as cloud:
        nlb = NLB("NLB", )
        middleware_cloud = EKS("Middleware")
        with Cluster("NetApp") as k8s:
            relay = Pod("Relay")
            netapps = [Pod("SLAM"), Pod("Object Detection"), Pod("Teleoperation")]
        nlb >> middleware_cloud >> relay >> netapps

    with Cluster("Edge") as edge:
        with Cluster("URLLC/EMBB Slice"):
            with Cluster("Core Network") as core:
                amarisoft = Custom("Amarisoft", "../img/amarisoft-testbed.png")
            nginx = Nginx("SSL")

        middleware_edge = Custom("Middleware", "../img/k8s.png")
        with Cluster("NetApp") as k8s_local:
            relay = Pod("Relay")
            netapps = [Pod("SLAM"), Pod("Object Detection"), Pod("Teleoperation")]

        nginx >> middleware_edge >> relay >> netapps
        middleware_edge >> amarisoft

    robot >> [nlb, nginx]
    #robot >> middleware_cloud >> net_app2
    #net_app >> middleware_cloud
