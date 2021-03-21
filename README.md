# RL-for-Self-Driving-Car-using-CARLA
Our team's research project as part of CS 748, Advances in Intelligent and Learning Agents (Spring 2021).  
Using online methods of Reinforcement Learning to train Self Driving Cars on the CARLA simulator.



## Setting up
Clone this repo and download [CARLA_0.9.9.4.tar.gz](https://drive.google.com/open?id=1p5qdXU4hVS2k5BOYSlEm7v7_ez3Et9bP) from the CARLA  downloads page.  
Copy the folders listed in .gitignore from the untarred CARLA directory to the main directory of this repo.


## Running CARLA

Open three seperate bash terminals and execute the following seperately and serially

```sh
./CarlaUE4.sh -windowed -opengl -quality-level=Low
```

```sh
cd PythonAPI/util
python3 config.py --map Town01 --weather ClearNoon
```

```sh
cd PythonAPI/examples
python3 manual_control.py             #(for navigating the map manually)
python3 Qlearning_manual_control.py   #(for starting the model training)
```


