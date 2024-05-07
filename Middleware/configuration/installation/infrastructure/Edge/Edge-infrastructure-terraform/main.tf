terraform {
  required_providers {
    helm = {
      source = "hashicorp/helm"
      version = "2.12.1"
    }
  }
}

provider "helm" {
  kubernetes {
    config_path = "~/.kube/config"  
  }
}

resource "helm_release" "redis" {
  name       = "redis"
  namespace  = "redis"
  create_namespace = true
  repository = "https://charts.bitnami.com/bitnami"
  chart      = "redis"
  version    = "17.1.4"

  values = [
    file("${path.module}/redis-values.yaml")
  ]
}

#resource "helm_release" "ingress-nginx" {
#  name       = "ingress-nginx"
#  namespace  = "ingress-nginx"
#  create_namespace = true
#  repository = "https://kubernetes.github.io/ingress-nginx"
#  chart      = "ingress-nginx"
#  version    = "4.8.3"
#
#  values = [
#    file("${path.module}/nginx-values.yaml")
#  ]
#}

resource "helm_release" "influxdb" {
  name       = "influxdb"
  namespace  = "influxdb"
  create_namespace = true
  repository = "https://helm.influxdata.com/"
  chart      = "influxdb"
  version    = "4.12.5"

  values = [
    file("${path.module}/influxdb-values.yaml")
  ]
}

resource "helm_release" "loki-stack" {
  name       = "loki"
  namespace  = "loki"
  create_namespace = true
  repository = "https://grafana.github.io/helm-charts"
  chart      = "loki-stack"
  version    = "2.10.0"

  values = [
    file("${path.module}/loki-values.yaml")
  ]
}

resource "helm_release" "rabbitmq" {
  name       = "rabbitmq"
  namespace  = "rabbitmq"
  create_namespace = true
  repository = "https://charts.bitnami.com/bitnami"
  chart      = "rabbitmq"
  version    = "12.13.1"

  values = [
    file("${path.module}/rabbitmq-values.yaml")
  ]
}