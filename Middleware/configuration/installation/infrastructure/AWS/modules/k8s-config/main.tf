locals {
  namespace = "middleware"
  central-api-namespace = "central-api"
}

data "aws_eks_cluster" "cluster" {
  name = var.cluster_name  
}


resource "kubernetes_namespace" "middleware" {
  metadata {
    name = local.namespace
    annotations = {
      name = local.namespace
    }
  }
  
}

resource "kubernetes_namespace" "central-api" {
  metadata {
    name = local.central-api-namespace
    annotations = {
      name = local.central-api-namespace
    }
  }
  
}

resource "kubernetes_namespace" "redis" {
  metadata {
    name = "redis"
    annotations = {
      name = "redis"
    }
  }
  
}
resource "kubernetes_namespace" "memgraph" {
  metadata {
    name = "memgraph"
    annotations = {
      name = "memgraph"
    }
  }
  
}

resource "kubernetes_service_account" "orchestrator" {

  metadata {
    namespace = local.namespace
    name      = var.service_account_name
    annotations = {
      "eks.amazonaws.com/role-arn" = var.middleware_role_arn
    }
  }
  automount_service_account_token = true
}

resource "kubernetes_cluster_role" "admin" {
  metadata {
    name      = "admin-role"    
  }
  rule {
    api_groups = ["*"]
    resources = ["*"]
    verbs = ["*"]
  }
}
resource "kubernetes_cluster_role_binding" "admin_role_binding" {
  metadata {
    name = "admin-role-binding"    
  }
  role_ref {
    api_group = "rbac.authorization.k8s.io"
    kind = "ClusterRole"
    name = "admin-role"
  }
  subject {
    kind = "User"
    api_group = "rbac.authorization.k8s.io"
    name = var.aws_iam_role_arn
  }
}

resource "kubernetes_role" "role" {
  metadata {
    name      = "${var.service_account_name}-role"
    namespace = local.namespace
  }
  rule {
    api_groups = ["", "apps", "batch"]
    resources = ["services",
      "nodes",
      "namespaces",
      "pods",
      "deployments",
      "daemonsets",
      "statefulsets",
      "replicasets",
      "jobs",
      "pods/log",
    "pods/exec"]
    verbs = ["get", "list", "watch", "create", "delete"]
  }
}

resource "kubernetes_role_binding" "role_binding" {
  metadata {
    name = "${var.service_account_name}-role-binding"
    namespace = local.namespace
  }
  role_ref {
    api_group = "rbac.authorization.k8s.io"
    kind = "Role"
    name = kubernetes_role.role.metadata[0].name
  }
  subject {
    kind = "ServiceAccount"
    namespace = local.namespace
    name = var.service_account_name
  }
}