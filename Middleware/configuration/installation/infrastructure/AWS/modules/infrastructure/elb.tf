resource "aws_lb" "middldeware-lb" {
  name               = "middleware-lb"
  internal           = false
  load_balancer_type = "network"
  subnets            = module.vpc.public_subnets

  enable_deletion_protection = false

  tags = {
    terraform = "true"
  }
}

resource "aws_lb_target_group" "middleware-tg" {
  name     = "middleware-tg"
  port     = 31000
  protocol = "TCP"
  vpc_id   = module.vpc.vpc_id
}



resource "aws_lb_target_group" "redis-tg" {
  name     = "redis-tg"
  port     = 31001
  protocol = "TCP"
  vpc_id   = module.vpc.vpc_id
}

resource "aws_lb_target_group" "memgraph-tg" {
  name     = "memgraph-tg"
  port     = 31007
  protocol = "TCP"
  vpc_id   = module.vpc.vpc_id
}

resource "aws_lb_target_group" "central-api-tg" {
  name     = "central-api-tg"
  port     = 31003
  protocol = "TCP"
  vpc_id   = module.vpc.vpc_id
}

resource "aws_lb_listener" "front_end" {
  load_balancer_arn = aws_lb.middldeware-lb.arn
  port              = 80
  protocol          = "TCP"
  default_action {
    type             = "forward"
    target_group_arn = aws_lb_target_group.middleware-tg.arn
  }
}

resource "aws_lb_listener" "redis" {
  load_balancer_arn = aws_lb.middldeware-lb.arn
  port              = 6379
  protocol          = "TCP"  

  default_action {
    type             = "forward"
    target_group_arn = aws_lb_target_group.redis-tg.arn
  }
}

# this port routing is added for the old middleware versions to connect to
resource "aws_lb_listener" "redis-obsolete" {
  load_balancer_arn = aws_lb.middldeware-lb.arn
  port              = 6380
  protocol          = "TCP"  

  default_action {
    type             = "forward"
    target_group_arn = aws_lb_target_group.redis-tg.arn
  }
}

resource "aws_lb_listener" "memgraph" {
  load_balancer_arn = aws_lb.middldeware-lb.arn
  port              = 7687
  protocol          = "TCP"    

  default_action {
    type             = "forward"
    target_group_arn = aws_lb_target_group.memgraph-tg.arn
  }
}

resource "aws_lb_listener" "central-api" {
  load_balancer_arn = aws_lb.middldeware-lb.arn
  port              = 3000
  protocol          = "TCP"  

  default_action {
    type             = "forward"
    target_group_arn = aws_lb_target_group.central-api-tg.arn
  }
}

resource "aws_autoscaling_attachment" "middleware-asg-lb-mw" {
  autoscaling_group_name = module.eks_managed_node_group.node_group_autoscaling_group_names[0]
  lb_target_group_arn    = aws_lb_target_group.middleware-tg.arn

  #depends_on = [ module.eks_managed_node_group, aws_lb.middldeware-lb]
}
resource "aws_autoscaling_attachment" "middleware-asg-lb-redis" {
  autoscaling_group_name = module.eks_managed_node_group.node_group_autoscaling_group_names[0]
  lb_target_group_arn    = aws_lb_target_group.redis-tg.arn

  #depends_on = [ module.eks_managed_node_group, aws_lb.middldeware-lb]
}

resource "aws_autoscaling_attachment" "middleware-asg-lb-memgraph" {
  autoscaling_group_name = module.eks_managed_node_group.node_group_autoscaling_group_names[0]
  lb_target_group_arn    = aws_lb_target_group.memgraph-tg.arn

  #depends_on = [ module.eks_managed_node_group, aws_lb.middldeware-lb]
}

resource "aws_autoscaling_attachment" "middleware-asg-lb-central-api" {
  autoscaling_group_name = module.eks_managed_node_group.node_group_autoscaling_group_names[0]
  lb_target_group_arn    = aws_lb_target_group.central-api-tg.arn

  #depends_on = [ module.eks_managed_node_group, aws_lb.middldeware-lb]
}
