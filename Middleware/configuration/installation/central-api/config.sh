#!/bin/sh
kubectl create ns middleware-central

kubectl apply -f central-api-secret.yaml -n middleware-central
kubectl apply -f central-api-service.yaml -n middleware-central
kubectl apply -f central-api.yaml -n middleware-central
