#!/bin/bash

# Install conda dependencies
conda env create
conda activate pypowertrain

# Start Jupyter Notebook
jupyter notebook --port=8000 --allow-root
