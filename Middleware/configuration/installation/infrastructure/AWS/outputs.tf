output "cluster_endpoint" {
  description = "Endpoint for EKS control plane"
  value       = module.infrastructure.cluster_endpoint
}

output "cluster_ca_certificate" {
  description = "Base64 encoded certificate data required to communicate with the cluster"
  value       = module.infrastructure.cluster_ca_certificate
}

output "cluster_name" {
  description = "Name of the created K8s cluster"
  value       = module.infrastructure.cluster_name
}

output "cluster_security_group_id" {
  description = "Security group ids attached to the cluster control plane"
  value       = module.infrastructure.cluster_security_group_id
}

output "region" {
  description = "AWS region"
  value       = var.region
}

output "middleware_address" {
  description = "Fully qualified domain name under which the Middleware is accessible"
  value       = module.infrastructure.middleware_address
}

output "middleware_service_account_name" {
  value = module.cluster_config.service_account_name
}