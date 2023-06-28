#! /bin/bash
echo "Building development image for the plate measurement demonstrator"

docker build . -f dev.Dockerfile -t visir/plate_measurement_demonstrator_dev