resource "aws_iam_policy" "middleware_iam_policy" {
  name = var.iam_policy_name

  policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Effect = "Allow"
        Action = [
          "eks:AccessKubernetesApi",
          "secretsmanager:ListSecrets",
          "secretsmanager:GetSecretValue"
        ]
        Resource = "*"
      }
    ]
  })
}