#!/usr/bin/env bash

docker build -t mhseals/roboboat-2023:base -f Dockerfile .
docker push mhseals/roboboat-2023:base

docker build -t mhseals/roboboat-2023:nvidia -f Dockerfile.nvidia .
docker push mhseals/roboboat-2023:nvidia
