#!/bin/bash
set -e

rosdep update

exec "$@"
