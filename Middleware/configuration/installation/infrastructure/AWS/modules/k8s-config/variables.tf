variable "service_account_name" {
  description = "Name of the service account"
  type        = string
  default     = "orchestrator"
}

variable "region" {
  description = "AWS region"
  type        = string
  default     = "eu-west-2"
}

variable "cluster_endpoint" {
  description = "Endpoint for EKS control plane"  
}

variable "cluster_name" {
  description = "Kubernetes Cluster Name"
}

variable "cluster_ca_certificate" {
  description = "Base64 encoded certificate of the existing cluster"
}
variable "middleware_role_arn" {
  description = "ARN of the middleware role to be associated with the service account"
}
variable "aws_iam_role_arn" {
  description = "ARN of IAM Role that has permissions to manage the cluster for other IAM users"
  type        = string  
}