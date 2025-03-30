#!/bin/bash

# Create a Python virtual environment
# python3 -m venv venv

# Activate the virtual environment
# source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
pip install -r .docsrc/sphinx_requirements.txt
pip install ./rs_protocol/
pip install ./bplprotocol/  # TODO: Remove this when bplprotocol is depreciated 