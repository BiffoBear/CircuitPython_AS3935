#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2021 Martin Stephens
#
# SPDX-License-Identifier: MIT
# Build script for running black, pylint and sphinx before major commits
black .
pylint biffobear_as3935.py
cd docs
sphinx-build -E -W -b html  . _build/html
cd ..
