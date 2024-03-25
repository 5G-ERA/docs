from diagrams import Cluster, Diagram
from diagrams.custom import Custom
from diagrams.k8s.compute import Deploy
from diagrams.k8s.infra import Node
from diagrams.onprem.inmemory import Redis

with Diagram("", direction="TB", show=False, filename="central-api"):
    robot = Custom("Robot", "../img/summit-xl.jpg")
    with Cluster("CentralApi") as central_api:
        central_api_pod = Deploy("CentralApi")
    with Cluster("Infrastructure") as infra:
        redis = Redis()
    with Cluster("Middleware Cloud") as mw_cloud:
         mw1 = Node()
    with Cluster("Middleware Edge") as mw_edge:
        mw2 = Node()
    [mw1, mw2] >> central_api_pod
    robot >> central_api_pod >> redis