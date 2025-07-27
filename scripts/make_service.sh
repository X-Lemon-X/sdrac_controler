#!/bin/bash

services=(
  can_init.service
  sdrac.service
)


dir="$(dirname "$0")"
cd "$dir/../services"
echo "Services to start: ${services[*]}"

for svc in "${services[@]}"; do
  sudo cp "$svc" /etc/systemd/system
done

sudo systemctl daemon-reload

for svc in "${services[@]}"; do
  sudo systemctl enable $svc
done
