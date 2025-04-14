# Acrobot

The Acrobot environment is based on Sutton’s work in “Generalization in Reinforcement Learning: Successful Examples Using Sparse Coarse Coding” and Sutton and Barto’s book. The system consists of two links connected linearly to form a chain, with one end of the chain fixed. The joint between the two links is actuated. The goal is to apply torque to the actuated pivot so that the free end of the linear chain moves to a vertical position, starting from an initial downward hanging state.

[](https://github.com/antonbolychev/acm2025-wasserschwein-acrobot/acrobot_animation.gif.mp4)

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

After creating the environment with all dependencies you need to clone [acm2025-wasserschwein-acrobot](https://github.com/antonbolychev/acm2025-wasserschwein-acrobot) repository:

```bash
$ git clone https://github.com/antonbolychev/acm2025-wasserschwein-acrobot
```

## Run

Run the [acrobot.py](https://github.com/antonbolychev/acm2025-wasserschwein-acrobot/acrobot.py) file:

```bash
cd <PATH_TO_REPO>
python acrobat.py
```

# Authors
* [Egor Miroshnichenko](https://github.com/Chenkomirosh)
* [Anton Bolychev](https://github.com/antonbolychev)
* [Vladislav Sarmatin](https://github.com/VladSarm)
* [Arsenii Shavrin](https://github.com/ArseniiSh)

# References
* [Sutton, R. S. (1996). Generalization in Reinforcement Learning: Successful Examples Using Sparse Coarse Coding. In D. Touretzky, M. C. Mozer, & M. Hasselmo (Eds.), Advances in Neural Information Processing Systems (Vol. 8). MIT Press.](https://proceedings.neurips.cc/paper/1995/file/8f1d43620bc6bb580df6e80b0dc05c48-Paper.pdf
)
* Sutton, R. S., Barto, A. G. (2018 ). Reinforcement Learning: An Introduction. The MIT Press.
<!-- ## Install

If you don't have uv install

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

then run 

```
uv run acrobot.py
``` -->
