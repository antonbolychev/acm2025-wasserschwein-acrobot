# Acrobot

The Acrobot environment is based on Sutton’s work in “Generalization in Reinforcement Learning: Successful Examples Using Sparse Coarse Coding” and Sutton and Barto’s book. The system consists of two links connected linearly to form a chain, with one end of the chain fixed. The joint between the two links is actuated. The goal is to apply torque to the actuated pivot so that the free end of the linear chain moves to a vertical position, starting from an initial downward hanging state.

# User Guide

## Installation and dependencies:

All necessary dependencies are in file [requirements.txt](https://github.com/antonbolychev/acm2025-wasserschwein-acrobot/requirements.txt)

Make the environment (pipenv or conda): \
`pip`: 
```bash
$ python -m venv your_env
$ source your_env/bin/activate
```
`conda`: 
```bash
$ conda create -n your_env
$ conda activate your_env
```
Then, install the requirements: 
```bash
$ pip install -r requirements.txt
```

To check: 
```bash
$ pip freeze
```
or 
```
$ conda list
```





## Install

If you don't have uv install

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

then run 

```
uv run acrobot.py
```
