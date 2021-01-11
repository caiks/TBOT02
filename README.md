# TBOT02 - TurtleBot3 dynamic controller 

[TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) is a [Robot Operating System](https://www.ros.org/about-ros/) standard platform robot. Here we extend the developments in [TBOT01](https://github.com/caiks/TBOT01#readme) to implement dynamic *modelling*. That is, instead of (1) acquiring *history*, (2) *modelling*, and (3) *applying model* in three different stages, all are done concurrently.

## Sections

[Download, build and run main executable](#main)

[Download, build and run TurtleBot3 nodes](#controller)

[Discussion](#Discussion)

<a name="main"></a>

## Download, build and run main executable

To run the non-ROS main executable it is only necessary to install the [AlignmentActive repository](https://github.com/caiks/AlignmentActive), the [AlignmentRepaC repository](https://github.com/caiks/AlignmentRepaC) and the underlying repositories. The `AlignmentActive` and the `AlignmentRepaC` modules require [modern C++](https://en.cppreference.com/w/) version 17 or later to be installed.

For example, in Ubuntu bionic (18.04),
```
sudo apt-get update -y && sudo apt install -y git g++ cmake

```
Then download the zip files or use git to get the `TBOT02` repository and the underlying `rapidjson`, `AlignmentC`, `AlignmentRepaC`and `AlignmentActive`  repositories -
```
cd
git clone https://github.com/Tencent/rapidjson.git
git clone https://github.com/caiks/AlignmentC.git
git clone https://github.com/caiks/AlignmentRepaC.git
git clone https://github.com/caiks/AlignmentActive.git
git clone https://github.com/caiks/TBOT02.git

```
Then download the [TBOT02 workspace repository](https://github.com/caiks/TBOT02_ws) -
```
git clone https://github.com/caiks/TBOT02_ws.git

cd ~/TBOT02_ws
cat data009a* >data009.bin

```
Then build -
```
cd
cp TBOT02/CMakeLists_noros.txt TBOT02/CMakeLists.txt
mkdir -p TBOT02_build
cd TBOT02_build
cmake -DCMAKE_BUILD_TYPE=RELEASE ../TBOT02
make

```
The `main` executable has various modes,
```
cd ../TBOT02_ws
ln -s ../TBOT02_build/main main

```
<a name = "controller"></a>

## Download, build and run TurtleBot3 nodes

To run the turtlebot it is necessary to install [ROS2](https://index.ros.org/doc/ros2/), [Gazebo](http://gazebosim.org/tutorials?cat=install) and [TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_simulation/#simulation) on a machine with a GPU and at least 4GB of memory.

[AWS EC2 instance](https://github.com/caiks/TBOT01#AWS)

[Windows 10 WSL2 instance](https://github.com/caiks/TBOT01#Windows)

[Installation](#Installation)
 
<a name = "Installation"></a>

### Installation

Now install Gazebo9,
```
sudo apt-get install -y gazebo9 libgazebo9-dev

gazebo -v

```
Install ROS2 Eloquent,
```
sudo apt install -y curl gnupg2 lsb-release

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update

sudo apt install -y ros-eloquent-desktop ros-eloquent-gazebo-* ros-eloquent-cartographer ros-eloquent-cartographer-ros

echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc
source ~/.bashrc

```
Test by running these nodes in separate shells,
```
ros2 run demo_nodes_cpp talker

ros2 run demo_nodes_py listener

```
Install TurtleBot3,

```
sudo apt install -y python3-argcomplete python3-colcon-common-extensions google-mock libceres-dev liblua5.3-dev libboost-dev libboost-iostreams-dev libprotobuf-dev protobuf-compiler libcairo2-dev libpcl-dev python3-sphinx python3-vcstool

mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws
wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
vcs import src < turtlebot3.repos
colcon build --symlink-install

echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc

echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
source ~/.bashrc

export TB3_MODEL=burger
export TURTLEBOT3_MODEL=${TB3_MODEL}

```
To test launch one of these worlds,
```
ros2 launch turtlebot3_gazebo empty_world.launch.py

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

```
Then in a separate shell,
```
export TB3_MODEL=burger
export TURTLEBOT3_MODEL=${TB3_MODEL}

ros2 run turtlebot3_teleop teleop_keyboard

```
Check that you can steer the turtlebot using the w/x and a/d keys.

Now download and build the `TBOT02` repository and the underlying `rapidjson`, `AlignmentC`, `AlignmentRepaC` and `AlignmentActive` repositories -
```
cd ~/turtlebot3_ws/src

git clone https://github.com/Tencent/rapidjson.git
git clone https://github.com/caiks/AlignmentC.git
git clone https://github.com/caiks/AlignmentRepaC.git
git clone https://github.com/caiks/AlignmentActive.git
git clone https://github.com/caiks/TBOT02.git
git clone https://github.com/caiks/TBOT02_ws.git

cd ~/turtlebot3_ws/src/TBOT02_ws
cat data009a* >data009.bin

cd ~/turtlebot3_ws/src
mkdir -p AlignmentC_build AlignmentRepaC_build AlignmentActive_build
cd ~/turtlebot3_ws/src/AlignmentActive_build
cmake -DCMAKE_BUILD_TYPE=RELEASE ../AlignmentActive
make AlignmentC AlignmentRepaC AlignmentActive

cd ~/turtlebot3_ws/src/TBOT02
cp CMakeLists_ros.txt CMakeLists.txt

cd ~/turtlebot3_ws
colcon build --packages-select TBOT02

source ~/.bashrc
```

The simulation can be started in paused mode,
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/TBOT02_ws/gazebo_models

gazebo -u --verbose ~/turtlebot3_ws/src/TBOT02_ws/env001.model -s libgazebo_ros_init.so

```
In a separate shell,
```
cd ~/turtlebot3_ws/src/TBOT02_ws

ros2 run TBOT02 controller data.bin 250

```
Press play in `gazebo` and the turtlebot3 will start moving.

To run the non-ros `main` executable, create a link,

```
cd ~/turtlebot3_ws/src/TBOT02_ws
ln -s ~/TBOT02_build/main main

```

<a name = "Discussion"></a>

## Discussion

Now let us investigate various turtlebot *models* and controllers. 

[Dynamic modelling](#Dynamic)

[Actor node](#Actor)

<a name = "Dynamic"></a>

### Dynamic modelling

In the [TBOT01 section on motor actions](https://github.com/caiks/TBOT01#Motor) we ran the turtlebot for 12 hours to create an unbiased random-turn training dataset, `data009`, 
```
cd ~/turtlebot3_ws/src/TBOT01_ws

ros2 run TBOT01 controller data009.bin 250 5000 5000 

```
```
cd ~/TBOT01_ws

./main analyse data009
hr->dimension: 363
hr->size: 172301
({(<scan,1>,0)},692 % 1)
({(<scan,1>,1)},33376 % 1)
({(<scan,1>,2)},27585 % 1)
({(<scan,1>,3)},22370 % 1)
({(<scan,1>,4)},17382 % 1)
({(<scan,1>,5)},14909 % 1)
({(<scan,1>,6)},12968 % 1)
({(<scan,1>,7)},43019 % 1)

({(<scan,180>,0)},890 % 1)
({(<scan,180>,1)},19205 % 1)
({(<scan,180>,2)},25416 % 1)
({(<scan,180>,3)},23958 % 1)
({(<scan,180>,4)},18528 % 1)
({(<scan,180>,5)},16419 % 1)
({(<scan,180>,6)},14367 % 1)
({(<scan,180>,7)},53518 % 1)

({(motor,0)},17809 % 1)
({(motor,1)},136432 % 1)
({(motor,2)},18060 % 1)

({(location,door12)},2067 % 1)
({(location,door13)},2365 % 1)
({(location,door14)},2012 % 1)
({(location,door45)},1288 % 1)
({(location,door56)},2314 % 1)
({(location,room1)},42708 % 1)
({(location,room2)},19975 % 1)
({(location,room3)},17110 % 1)
({(location,room4)},45058 % 1)
({(location,room5)},16658 % 1)
({(location,room6)},20746 % 1)

({(position,centre)},41677 % 1)
({(position,corner)},38736 % 1)
({(position,side)},91888 % 1)
```
We then *induced* random regional *model* 26,
```
cd ~/TBOT01_ws

./main induce model026 8 >model026.log

./main entropy_region model026 1 data009
model: model026
mult: 1
dataset: data009
auto z = hr->size
z: 1723010
auto v = z * mult
v: 1723010
fudRepasSize(*dr->fud): 2209
frder(*dr->fud)->size(): 872
frund(*dr->fud)->size(): 60
treesSize(*dr->slices): 998
treesLeafElements(*dr->slices)->size(): 872
ent(*aa) * z: 8.93241e+06
ent(*bb) * v: 6.20557e+06
ent(*add(*aa,*bb)) * (z+v): 1.71028e+07
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 1.96486e+06
```
Both `data009` and `model026` have been copied over to the [TBOT02 workspace repository](https://github.com/caiks/TBOT02_ws).

Now let us consider how we might be able to *induce* a *model* with a similar *likelihood* dynamically, i.e. with a stream of incoming *events* in real time. This is the functionality which is implemented in the `AlignmentActive` repository.

The accumulated *history* of a dynamic system would eventually use impracticable amounts of memory, so we will only be able to keep a fixed maximum *size* of past *events*. A new *event* will be appended to the past *history* in sequence until the maximum *size* is reached. After that the new *event* will overwrite the oldest *event* using clock arithmetic.

In order to minimise the computation required to *induce* large *models*, only a leaf *slice* of past *events* will be selected, and the resultant *slice history* *shuffled*, in preparation for *fud induction* by the *layerer*. We must therefore keep a list of the leaf *slice variables* for each of the past *events* and the corresponding inverse map from leaf *slice variable* to its set of *events*. 

After an incoming *event* has been appended to the past *history*, the *model* will be *applied* to it to determine its *slice*. Instead of *applying* the entire *model*, only *fuds* along the *in-slice* path will be *applied*, thus avoiding the exponential increase in *application* time as the *models* become larger. Then the *slice* list and map will be updated with the new *slice variable*. If the current *slice size* exceeds a certain threshold the current *slice history* will be *modelled* by the *layerer*, and the resultant new *fud*, if any, will be added to the *model*. The *events* of the *slice history* will now have new *slice variables derived* from the *fud* and the parent *slice*, so the past *slice* list and map will be updated with the new children *slices*.

Together the structure of the *underlying history*, the *fud decomposition* and the *slice* list is called an active. The active also has references to *event* queues for each of the streams of incoming real-time *events*. The processing of new *events* in the *underlying event* queues is called update. The other main process is induce, in which new *model* is added when a leaf *slice size* exceeds the *induce* threshold. In addition, an active has a lock so that the active structure remains consistent during concurrent updates and induces.

We can see that the *slice* list is itself a compact *history* of the *slice variables* of the *model's* *slice* tree. So the active can also have a reference to an outgoing *event* queue for its own *slice history*. Now we can create *levels* of actives with the *underlying event* queues of higher *level* actives being either (a) the *event* queues of the *substrate* or (b) the *slice event* queues of lower *level* actives.

In order to allow different actives to update or induce concurrently, we must synchronise the issuing of new *variables* so that they are unique everywhere. To minimise the time waiting for locks, the *variables* are issued in blocks, so that an active need only request a new block of *variables* when the current block has been fully assigned. A common active *system* controls the issuing of the blocks.

Actives are designed to process chronological streams of *events*. The past *events* are known up to the *underlying history size*, so time-wise *frames* of past *events* can be added to the current *event* by remapping the *variables* of the past *frames*. In this way, dynamic *alignments* can be *modelled*. Although *modelling* of past *frames* is possible without actives, as we saw in [TBOT01](https://github.com/caiks/TBOT01#Timewise), this requires the duplication and remapping of the entire *model*, and a duplication of *underlying substrate*, for each *frame*. This duplication becomes impracticable for large *models*. In addition, actives also allow past *frames* of the *slice history* itself to be added to the current *event*. That is, recurrent past self *frames* are allowed. This recursive functionality would be difficult to implement without active *history*.

Given the current *slice*, the active *underlying history* enables us to look forward from each of the past *events* of the *slice* to see the succeeding *events*. We can therefore determine the consequences of different actions. That is, given some definition of a goal, we can calculate a goodness for each *value* of the action *variable*. We do this by selecting the subset of the *events* of the *slice* having that action *value* and then for each discounting by the time taken to the attainment of the goal. The sum of these forms the goodness for that action *value*. In this way we can choose the present motor action that is expected to be the quickest to obtain the goal, based on the *model's classification* of past experiences.

Now let us consider what the *induce slice* threshold should be. We *induced* a *fud* on a *history* of increasingly large *sizes* taken from the beginning of the random region *substrate* formed by `data009`, and calculated the *implied diagonal valency percent* and the *likelihood* of each,
```
cd ~/TBOT02_ws

./main fud_region model040_100 100
...
m: 2
a: 32.7146
z: 100
100.0*(exp(a/z/(m-1))-1.0): 38.7005
...

./main entropy_region model040_100 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 823871

./main fud_region model040_1000 1000
...
m: 2
a: 323.051
z: 1000
100.0*(exp(a/z/(m-1))-1.0): 38.1336
...

./main entropy_region model040_1000 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 866003

./main fud_region model040_9000 9000
...
m: 3
a: 5558.56
z: 9000
100.0*(exp(a/z/(m-1))-1.0): 36.1802
...

./main entropy_region model040_9000 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 891527

./main fud_region model040_10000 10000
...
m: 3
a: 5984.61
z: 10000
100.0*(exp(a/z/(m-1))-1.0): 34.882
...

./main entropy_region model040_10000 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 934363

./main fud_region model040_20000 20000
...
m: 2
a: 6224.28
z: 20000
100.0*(exp(a/z/(m-1))-1.0): 36.5081
...

./main entropy_region model040_20000 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 928491

./main fud_region model040_100000 100000
...
m: 3
a: 57391.3
z: 100000
100.0*(exp(a/z/(m-1))-1.0): 33.2366
...

./main entropy_region model040_100000 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 891731

./main fud_region model040_1000000 1000000
...
m: 3
a: 470580
z: 861505
100.0*(exp(a/z/(m-1))-1.0): 31.4051
...

./main entropy_region model040_1000000 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 775173
```

Size|Diagonal|Likelihood
---|---|---
100|38.7|823,871
1,000|38.1|866,003
9,000|36.2|891,527
10,000|34.9|934,363
20,000|36.5|928,491
100,000|33.2|891,731
861,505|31.4|775,173

The *implied diagonal valency percent* of the root *fud* of *model* 26 of *slice size* 861,505 was also 31.4.

We can see that for this parameterisation of the *fud induction* the optimal *slice size* is around 10,000. Indeed the *likelihoods* and *implied diagonals* of the smaller *sizes* are all greater than for the largest *size*. This is somewhat counter-intuitive, but suggests that dynamic *modelling* might not be at much of a disadvantage, if at all.

We then *induced* *model* 27 given *underlying model* 26,
```
cd ~/TBOT01_ws

./main induce model027 32 >model027.log

./main entropy model027 1 data009
model: model027
mult: 1
dataset: data009
auto z = hr->size
z: 172301
auto v = z * mult
v: 172301
fudRepasSize(*dr->fud): 52365
frder(*dr->fud)->size(): 9746
frund(*dr->fud)->size(): 360
treesSize(*dr->slices): 13841
treesLeafElements(*dr->slices)->size(): 9746
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 231911

./main observe data008 model027 data009 location
...
100.0*match_count/z: 63.9114
```
Now let us consider what the *slice* threshold should be for this *2-level system*,
```
cd ~/TBOT02_ws

./main fud model041_100 100
...
m: 4
a: 91.952
z: 100
100.0*(exp(a/z/(m-1))-1.0): 35.867
...

./main entropy model041_100 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 15081.6

./main fud model041_1000 1000
...
m: 4
a: 748.269
z: 1000
100.0*(exp(a/z/(m-1))-1.0): 28.3285
...

./main entropy model041_1000 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 14565

./main fud model041_2000 2000
...
m: 4
a: 1647.52
z: 2000
100.0*(exp(a/z/(m-1))-1.0): 31.5986
...

./main entropy model041_2000 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 61952.8

./main fud model041_3000 3000
...
m: 3
a: 1615.26
z: 3000
100.0*(exp(a/z/(m-1))-1.0): 30.8931
...

./main entropy model041_3000 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 61685.9

./main fud model041_5000 5000
...
m: 4
a: 4069.01
z: 5000
100.0*(exp(a/z/(m-1))-1.0): 31.1626
...

./main entropy model041_5000 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 53583.5
./main fud model041_10000 10000
...
m: 4
a: 8042.99
z: 10000
100.0*(exp(a/z/(m-1))-1.0): 30.7477
...

./main entropy model041_10000 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 54620.7

./main fud model041_20000 20000
...
m: 4
a: 18897.4
z: 20000
100.0*(exp(a/z/(m-1))-1.0): 37.02
...

./main entropy model041_20000 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 54580.6

./main fud model041_100000 100000
...
m: 4
a: 74101.8
z: 100000
100.0*(exp(a/z/(m-1))-1.0): 28.0187
...

./main entropy model041_100000 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 76387.3

./main fud model041_1000000 1000000
...
m: 4
a: 142948
z: 172301
100.0*(exp(a/z/(m-1))-1.0): 31.8568
...

./main entropy model041_1000000 1 data009
...
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 76076.7
```

Size|Diagonal|Likelihood
---|---|---
100|35.9|15,081
1,000|28.3|14,565
2,000|31.6|61,952
3,000|30.9|61,685
5,000|31.1|53,583
10,000|30.7|54,620
20,000|37.0|54,580
100,000|28.0|76,387
172,301|31.9|76,076

We can see that for the *2-level model* there is a jump in *likelihood* between 1,000 records and 2,000 records. There is another jump later on between 20,000 records and 100,000 records. Unlike the *1-level model* the *likelihood*  of the *2-level model* generally increases with *size*, at least at the root *fud*. The reason for this is probably because the *1-level-model* was *modelled* with non-sequential *history* - each  *event* consisted of 60 *scan variables* taken at random from the 360 of the *substrate*. The *2-level-model* is *modelled* with sequential *history*, so it is only after around 2,000 records or 8 minutes that all of the rooms have been visited. Although *active modelling* with small thresholds of a few hundred *events* may produce a root *fud* with a low *alignment*, that is not to say that a large *model* with many nodes, though lopsided, might not have high *likelihood* nonetheless. In general, however, a large initial threshold looks desirable.

Let us simulate the dynamic *modelling* of *model* 26. First note that *model* 26 was obtained from a non-sequential *history* of random regions of 60 degrees field-of-view. This *history* is not suitable for a stream of *events*, so we will consider only the fixed field-of-view between 330 degrees and 29 degrees. The threshold *slice size* is 7,000. This figure is chosen because the trailing *slice* in *model* 26 is 7,337.

In test `induce03` we stream the *events* of `data009` into a single active, updating and then inducing for each *event*,

```
cd ~/TBOT02_ws

/usr/bin/time -v ./main induce03 model043 data009 7000 >model043.log 2>&1
...
model043	update apply	event id: 0	history id: 0	slice: 0	slice size: 1	time 0.0002089s
model043	update apply	event id: 1	history id: 1	slice: 0	slice size: 2	time 5e-06s
...
model043	update apply	event id: 6998	history id: 6998	slice: 0	slice size: 6999	time 1.5e-06s
model043	update apply	event id: 6999	history id: 6999	slice: 0	slice size: 7000	time 1.6e-06s
model043	induce copy	slice: 0	slice size: 7000	repa dimension: 60	sparse capacity: 0	sparse paths: 0	variable: 65536	time 0.0016088s
model043	induce model	repa dimension: 60	sparse dimension: 0
model043	induce model	dimension: 60	size: 7000
model043	induce model	der vars algn density: 2389.93	impl bi-valency percent: 40.6942	der vars cardinality: 2	fud cardinality: 2
model043	induce model	time 0.0753538s
model043	induce update	slice: 0	parent slice: 0	children cardinality: 8	fud size: 10	fud cardinality: 1	model cardinality: 10	time 0.0017929s
model043	update apply	event id: 7000	history id: 7000	slice: 131072	slice size: 2350	time 8.8e-06s
model043	update apply	event id: 7001	history id: 7001	slice: 131072	slice size: 2351	time 5.1e-06s
...
model043	update apply	event id: 170899	history id: 170899	slice: 131333	slice size: 6999	time 4.7e-06s
model043	update apply	event id: 170900	history id: 170900	slice: 131333	slice size: 7000	time 4.9e-06s
model043	induce copy	slice: 131333	slice size: 7000	repa dimension: 60	sparse capacity: 0	sparse paths: 0	variable: 65888	time 0.0014435s
model043	induce model	repa dimension: 40	sparse dimension: 0
model043	induce model	dimension: 40	size: 7000
model043	induce model	der vars algn density: 594.221	impl bi-valency percent: 4.3358	der vars cardinality: 3	fud cardinality: 14
model043	induce model	time 0.393112s
model043	induce update	slice: 131333	parent slice: 0	children cardinality: 6	fud size: 20	fud cardinality: 36	model cardinality: 635	time 0.0038211s
model043	update apply	event id: 170901	history id: 170901	slice: 131339	slice size: 6824	time 3.3e-05s
model043	update apply	event id: 170902	history id: 170902	slice: 131339	slice size: 6825	time 1.26e-05s
...
model043	update apply	event id: 172203	history id: 172203	slice: 131327	slice size: 12204	time 4.75e-05s
model043	induce copy	slice: 131327	slice size: 12204	repa dimension: 60	sparse capacity: 0	sparse paths: 0	variable: 65902	time 0.0030067s
model043	induce model	repa dimension: 11	sparse dimension: 0
model043	induce model	dimension: 11	size: 12204
model043	induce model	no alignment
model043	induce model	time 0.0070914s
model043	induce update fail	slice: 131327	slice size: 12204	time 1.3e-06s
model043	update apply	event id: 172204	history id: 172204	slice: 131223	slice size: 166	time 1.77e-05s
model043	update apply	event id: 172205	history id: 172205	slice: 131283	slice size: 4813	time 1.38e-05s
...
model043	update apply	event id: 172299	history id: 172299	slice: 131266	slice size: 1366	time 9.4e-06s
model043	update apply	event id: 172300	history id: 172300	slice: 131266	slice size: 1367	time 4.3e-06s
```
We can see that at the 7000th *event* the induce threshold is crossed and the root *fud* is *modelled*, having a *diagonal* of `40.7`. *Model* 43 eventually has 36 *fuds*. The last *fud* only has a diagonal of `4.3`. Note that there is a very large *slice* of 12,204 *events* which has no *alignment* - presumably because the turtlebot is looking into empty space up to the range of the lidar.

After all of the *events* are streamed, the active is dumped and the *model* is saved separately as an `ApplicationRepa`.

The *likelihood* of *model* 43 with no *shuffle scaling* is 190,568 -
```
./main entropy model043 1 data009
model: model043
mult: 1
dataset: data009
auto z = hr->size
z: 172301
auto v = z * mult
v: 172301
fudRepasSize(*dr->fud): 635
frder(*dr->fud)->size(): 234
frund(*dr->fud)->size(): 60
treesSize(*dr->slices): 269
treesLeafElements(*dr->slices)->size(): 234
ent(*aa) * z: 682079
ent(*bb) * v: 511291
ent(*add(*aa,*bb)) * (z+v): 1.38394e+06
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 190568
```
This may be compared to a *likelihood* of 197,025 for *model* 26 with the same *scaling*,
```
./main entropy_region model026 1 data009 1
model: model026
mult: 1
dataset: data009
auto z = hr->size
z: 172301
auto v = z * mult
v: 172301
fudRepasSize(*dr->fud): 2209
frder(*dr->fud)->size(): 872
frund(*dr->fud)->size(): 60
treesSize(*dr->slices): 998
treesLeafElements(*dr->slices)->size(): 872
ent(*aa) * z: 892604
ent(*bb) * v: 620447
ent(*add(*aa,*bb)) * (z+v): 1.71008e+06
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 197025
```
That is, the dynamic *model* is nearly as *likely* as the static *model*.

To see the effect of the induce threshold, in test `induce04` we again stream the *events* of `data009` into a single active, but this time update all of them without induce. Then a single induce creates the entire *model*,

```
/usr/bin/time -v ./main induce04 model044 data009 7000 >model044.log 2>&1
...
model044	induce copy	slice: 0	slice size: 172301	repa dimension: 60	sparse capacity: 0	sparse paths: 0	variable: 65536	time 0.0498034s
model044	induce model	repa dimension: 60	sparse dimension: 0
model044	induce model	dimension: 60	size: 172301
model044	induce model	der vars algn density: 59404.8	impl bi-valency percent: 41.167	der vars cardinality: 2	fud cardinality: 2
model044	induce model	time 0.798991s
model044	induce update	slice: 0	parent slice: 0	children cardinality: 9	fud size: 11	fud cardinality: 1	model cardinality: 11	time 0.0377803s
...
model044	induce copy	slice: 131238	slice size: 12217	repa dimension: 60	sparse capacity: 0	sparse paths: 0	variable: 65794	time 0.0030488s
model044	induce model	repa dimension: 12	sparse dimension: 0
model044	induce model	dimension: 12	size: 12217
model044	induce model	no alignment
model044	induce model	time 0.0073799s
model044	induce update fail	slice: 131238	slice size: 12217	time 1.4e-06s
...
model044	induce copy	slice: 131348	slice size: 7004	repa dimension: 60	sparse capacity: 0	sparse paths: 0	variable: 65988	time 0.001416s
model044	induce model	repa dimension: 8	sparse dimension: 0
model044	induce model	dimension: 8	size: 7004
model044	induce model	der vars algn density: 7.61114	impl bi-valency percent: 0.054349	der vars cardinality: 3	fud cardinality: 3
model044	induce model	time 0.0188726s
model044	induce update	slice: 131348	parent slice: 0	children cardinality: 6	fud size: 9	fud cardinality: 36	model cardinality: 742	time 0.0019504s
model044	induce copy	slice: 131357	slice size: 7000	repa dimension: 60	sparse capacity: 0	sparse paths: 0	variable: 65991	time 0.0013985s
model044	induce model	repa dimension: 5	sparse dimension: 0
model044	induce model	dimension: 5	size: 7000
model044	induce model	no alignment
model044	induce model	time 0.0040473s
model044	induce update fail	slice: 131357	slice size: 7000	time 1.4e-06s
```
The root *fud* is *modelled* on the entire *history* of 172,301 *events*. It has a *diagonal* of `41.2`. The *model* 44 also has 36 *fuds*. The last *fud* only has a diagonal of `0.05`. Again, there is a very large *slice* of 12,217 *events* which has no *alignment*. 

The *likelihood* of *model* 44 is 194,298 -
```
./main entropy model044 1 data009
model: model044
mult: 1
dataset: data009
auto z = hr->size
z: 172301
auto v = z * mult
v: 172301
fudRepasSize(*dr->fud): 742
frder(*dr->fud)->size(): 252
frund(*dr->fud)->size(): 60
treesSize(*dr->slices): 287
treesLeafElements(*dr->slices)->size(): 252
ent(*aa) * z: 672865
ent(*bb) * v: 525898
ent(*add(*aa,*bb)) * (z+v): 1.39306e+06
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 194298
```
So *model* 44 is very similar to *model* 43, confirming that dynamic *modelling* is not much less efficient than static.

We can test a small induce threshold of 100. Now the *likelihood* is 221,664 -
```
/usr/bin/time -v ./main induce04 model045 data009 100 >model045.log 2>&1

./main entropy model045 1 data009
model: model045
mult: 1
dataset: data009
auto z = hr->size
z: 172301
auto v = z * mult
v: 172301
fudRepasSize(*dr->fud): 33249
frder(*dr->fud)->size(): 6853
frund(*dr->fud)->size(): 60
treesSize(*dr->slices): 8671
treesLeafElements(*dr->slices)->size(): 6853
ent(*aa) * z: 1.27283e+06
ent(*bb) * v: 717772
ent(*add(*aa,*bb)) * (z+v): 2.21227e+06
ent(*add(*aa,*bb)) * (z+v) - ent(*aa) * z - ent(*bb) * v: 221664
```

Let us simulate the dynamic *modelling* of the *2-level* *model* 27. In test `induce05` there are 12 *level* 1 actives each with a fixed non-overlapping field-of-view of 30 degrees, and 1 *level* 2 active. We begin with an induce threshold *slice size* of 1,000, except for an initial induce threshold *slice size* for *level* 2 of 30,000,
```
/usr/bin/time -v ./main induce05 model046 data009 1000 30000 >model046.log 2>&1

./main load05 model046 data009 12
model: model046
dataset: data009
level1Size: 12
hr->size: 172301
ur->listVarSizePair.size(): 366
model046_1_00   load    file name: model046_1_00.ac     time 0.0968937s
activeA.decomp->fudRepasSize: 2888
activeA.decomp->fuds.size(): 165
model046_1_01   load    file name: model046_1_01.ac     time 0.24568s
activeA.decomp->fudRepasSize: 2711
activeA.decomp->fuds.size(): 161
model046_1_02   load    file name: model046_1_02.ac     time 0.130942s
activeA.decomp->fudRepasSize: 2296
activeA.decomp->fuds.size(): 132
model046_1_03   load    file name: model046_1_03.ac     time 0.138418s
activeA.decomp->fudRepasSize: 2125
activeA.decomp->fuds.size(): 118
model046_1_04   load    file name: model046_1_04.ac     time 0.111514s
activeA.decomp->fudRepasSize: 2437
activeA.decomp->fuds.size(): 134
model046_1_05   load    file name: model046_1_05.ac     time 0.123201s
activeA.decomp->fudRepasSize: 2702
activeA.decomp->fuds.size(): 165
model046_1_06   load    file name: model046_1_06.ac     time 0.0853729s
activeA.decomp->fudRepasSize: 2879
activeA.decomp->fuds.size(): 167
model046_1_07   load    file name: model046_1_07.ac     time 0.0832004s
activeA.decomp->fudRepasSize: 2767
activeA.decomp->fuds.size(): 162
model046_1_08   load    file name: model046_1_08.ac     time 0.114457s
activeA.decomp->fudRepasSize: 2432
activeA.decomp->fuds.size(): 143
model046_1_09   load    file name: model046_1_09.ac     time 0.0596965s
activeA.decomp->fudRepasSize: 2264
activeA.decomp->fuds.size(): 126
model046_1_10   load    file name: model046_1_10.ac     time 0.0436621s
activeA.decomp->fudRepasSize: 2605
activeA.decomp->fuds.size(): 147
model046_1_11   load    file name: model046_1_11.ac     time 0.0522163s
activeA.decomp->fudRepasSize: 2595
activeA.decomp->fuds.size(): 165
model046_2      load    file name: model046_2.ac        time 0.0581269s
activeA.underlyingEventUpdateds: {172300}
activeA.historySize: 172301
activeA.historyOverflow: true
activeA.historyEvent: 0
activeA.decomp->fudRepasSize: 2752
activeA.decomp->fuds.size(): 139
fudRepasSize(*er1->fud): 30701
frder(*er1->fud)->size(): 9183
frund(*er1->fud)->size(): 360
treesSize(*er1->slices): 10956
treesLeafElements(*er1->slices)->size(): 9183
fudRepasSize(*er2->fud): 2752
frder(*er2->fud)->size(): 1243
frund(*er2->fud)->size(): 349
treesSize(*er2->slices): 1381
treesLeafElements(*er2->slices)->size(): 1243
fudRepasSize(*er3->fud): 6299
frder(*er3->fud)->size(): 1243
frund(*er3->fud)->size(): 360
treesSize(*er3->slices): 1381
treesLeafElements(*er3->slices)->size(): 1243
a: 1.03356e+06
b: 496376
c: 1.73741e+06
likelihood c-a-b: 207472
```
We can see that each of the *level* 1 actives has around 150 *fuds*. The *level* 1 active has 139 *fuds*. The overall *likelihood* of *model* 46 is 207,472. This may be compared to a *likelihood* for *model* 27 of 231,911. *Model* 27 has 4096 *level 2 fuds*.

In *model* 49, the *level* 1 actives have an initial threshold of 10,000 and thereafter 200, and the *level* 2 active has an initial threshold of 30,000 and thereafter 50,
```
/usr/bin/time -v ./main induce05 model049 data009 50 30000 12 1 200 10000 >model049.log 2>&1

./main load05 model049 data009 12
model: model049
dataset: data009
level1Size: 12
hr->size: 172301
ur->listVarSizePair.size(): 366
model049_1_00   load    file name: model049_1_00.ac     time 0.0641369s
activeA.decomp->fudRepasSize: 12308
activeA.decomp->fuds.size(): 738
model049_1_01   load    file name: model049_1_01.ac     time 0.159909s
activeA.decomp->fudRepasSize: 11693
activeA.decomp->fuds.size(): 699
model049_1_02   load    file name: model049_1_02.ac     time 0.0737249s
activeA.decomp->fudRepasSize: 11170
activeA.decomp->fuds.size(): 643
model049_1_03   load    file name: model049_1_03.ac     time 0.0758972s
activeA.decomp->fudRepasSize: 10004
activeA.decomp->fuds.size(): 566
model049_1_04   load    file name: model049_1_04.ac     time 0.0889874s
activeA.decomp->fudRepasSize: 10844
activeA.decomp->fuds.size(): 643
model049_1_05   load    file name: model049_1_05.ac     time 0.07918s
activeA.decomp->fudRepasSize: 12524
activeA.decomp->fuds.size(): 744
model049_1_06   load    file name: model049_1_06.ac     time 0.0731357s
activeA.decomp->fudRepasSize: 13824
activeA.decomp->fuds.size(): 787
model049_1_07   load    file name: model049_1_07.ac     time 0.085181s
activeA.decomp->fudRepasSize: 11356
activeA.decomp->fuds.size(): 726
model049_1_08   load    file name: model049_1_08.ac     time 0.074666s
activeA.decomp->fudRepasSize: 10267
activeA.decomp->fuds.size(): 619
model049_1_09   load    file name: model049_1_09.ac     time 0.0631516s
activeA.decomp->fudRepasSize: 9754
activeA.decomp->fuds.size(): 546
model049_1_10   load    file name: model049_1_10.ac     time 0.0782104s
activeA.decomp->fudRepasSize: 10985
activeA.decomp->fuds.size(): 645
model049_1_11   load    file name: model049_1_11.ac     time 0.0947589s
activeA.decomp->fudRepasSize: 11778
activeA.decomp->fuds.size(): 695
model049_2      load    file name: model049_2.ac        time 0.171182s
activeA.underlyingEventUpdateds: {172300}
activeA.historySize: 172301
activeA.historyOverflow: true
activeA.historyEvent: 0
activeA.decomp->fudRepasSize: 53407
activeA.decomp->fuds.size(): 3287
fudRepasSize(*er1->fud): 136507
frder(*er1->fud)->size(): 29518
frund(*er1->fud)->size(): 360
treesSize(*er1->slices): 37557
treesLeafElements(*er1->slices)->size(): 29518
fudRepasSize(*er2->fud): 53407
frder(*er2->fud)->size(): 8917
frund(*er2->fud)->size(): 7709
treesSize(*er2->slices): 12203
treesLeafElements(*er2->slices)->size(): 8917
fudRepasSize(*er3->fud): 116431
frder(*er3->fud)->size(): 8917
frund(*er3->fud)->size(): 360
treesSize(*er3->slices): 12203
treesLeafElements(*er3->slices)->size(): 8917
a: 1.49688e+06
b: 618212
c: 2.33977e+06
likelihood c-a-b: 224685
```
*Model* 49 has 3287 *fuds*. The *likelihood* of 224,685 is closer to that of *model* 27. The *model* 27 trailing *slice size* is 62. This demonstrates that *multi-level* dynamic *modelling* is also not much less efficient than static.

Now let us see the effect of adding *frames*. First we will consider the *1-level* static *model* 26 (*likelihood* 197,025) and corresponding dynamic *model* 43 (*likelihood* 190,568) again. Without any *frames*, test `induce07` reproduces *model* 43,

```
cd ~/TBOT02_ws
./main induce07 model052 data009 7000
...
frame01: 0
frame02: 0
frame03: 0
self01: 0
self02: 0
self03: 0
...
likelihood c-a-b: 190568
...
```
If we add one *frame* 12 *events* (or 3 seconds) before the present, we see a small increase in *likelihood* to 191,362 -
```
./main induce07 model052 data009 7000 0 12
...
frame01: 0
frame02: 12
frame03: 0
self01: 0
self02: 0
self03: 0
...
likelihood c-a-b: 191362
...
```
But adding two *frames* appears to reduce it to 190,965 -
```
./main induce07 model052 data009 7000 0 6 12
...
frame01: 0
frame02: 6
frame03: 12
self01: 0
self02: 0
self03: 0
...
likelihood c-a-b: 190965
```
We can continue to experiment with up to 3 *underlying frames* and up to 3 reflexive *frames*, summarised in this table,

frame 1|frame 2|frame 3|self 1|self 2|self 3|likelihood
---|---|---|---|---|---|---
0||||||190,568
0|12|||||191,362
0|6|12||||190,965
0|1|12||||188,684
12|13|14||||190,415
0|1|2||||189,463
0|||2|||191,337
0|||12|||188,750
0|1|3|6|10|15|195,871
0|1|3|6|10||195,580
0|1|3|6|||191,785
0|1|3||||191,370
0|1|||||180,998
0|2|||||169,789
0|4|||||172,005
0|8|||||176,058
0|16|||||192,368
0|32|||||196,969
0|||6|10|15|191,851
0|2|3|8|16|32|192,746
0|16|32|8|16|32|194,707
0|16|32||||196,024
0|48|||||193,726
0|32||2|||195,051
0|32||32|||191,185
0|||32|||191,018
0|32||6|||196,731
0|32||6|10||195,652
0|32||6|10|15|198,154
0|32|48|6|10|15|197,487
0|32|0|8|16|24|196,708
0|32||10|15||197,026

These are the key results -
frame 1|frame 2|frame 3|self 1|self 2|self 3|likelihood
---|---|---|---|---|---|---
0||||||190,568
0|1|3|6|10|15|195,871
0|32|||||196,969
0|32||6|10|15|198,154

We can see in the case of *frames* of 0 (now), 1 (0.25s) and 3 (0.75s) and self *frames* of 6 (1.5s), 10 (2.5s) and 15 (3.75s), there is an increase to a *likelihood* of 195,871. This seems to pick up the turtlebot's approach, turn and rebound from a wall. Separately there appears to be dynamic *alignment* between *frame* 0 (now) and *frame* 32 (8s), perhaps because of a common spacing between turns - a 'resonance' of the house dimensions. This dynamic *alignment* appears to be enhanced if we include the self *frames* of 6 (1.5s), 10 (2.5s) and 15 (3.75s). In general, self *frames* appear to be most interesting at longer times, whereas *underlying frames* appear to be optimised at specific times. We can conjecture that self *frames* are more general or contextual and less sensitive to their exact placement. *Underlying frames* work at particular dynamic geometries.

Lastly, consider the effect of *frames* on the *2-level model* 49,
```
/usr/bin/time -v ./main induce08 model056 data009 50 30000 12 1 200 10000 0 1 3 6 10 15 >model056.log 2>&1
...
likelihood c-a-b: 225360
...

/usr/bin/time -v ./main induce08 model057 data009 50 30000 12 1 200 10000 0 32 0 6 10 15 >model057.log 2>&1
...
likelihood c-a-b: 223097
...

/usr/bin/time -v ./main induce08 model058 data009 50 30000 12 1 200 10000 0 32 >model058.log 2>&1
...
likelihood c-a-b: 221891
...
...
```
Examination of the logs for *models* 56 and 57 show a lot of very low *alignment slices*. This suggests that the initial threshold is too high, reducing the *underlying variable* cardinality. Re-running with lower threshold alters the results -
```
/usr/bin/time -v ./main induce08 model059 data009 50 500 12 1 200 200 0 1 3 6 10 15 >model059.log 2>&1
...
likelihood c-a-b: 227,495
...

/usr/bin/time -v ./main induce08 model060 data009 50 500 12 1 200 200 0 32 0 6 10 15 >model060.log 2>&1
...
likelihood c-a-b: 212,350
...
```
This shows the set of best cases,
frame 1|frame 2|frame 3|self 1|self 2|self 3|likelihood
---|---|---|---|---|---|---
0||||||224,685
0|1|3|6|10|15|227,495
0|32||6|10|15|223,097
0|32|||||221,891

The *likelihood* of *model* 49 is 224,685. 
In the case of *frames* of 0 (now), 1 (0.25s) and 3 (0.75s) and self *frames* of 6 (1.5s), 10 (2.5s) and 15 (3.75s), there is a small increase to a *likelihood* of 227,495. That was the only case of an increase, but we can still conclude that there is some dynamic *alignment* in the *2-level* case too.

<a name = "Actor"></a>

### Actor node

The `TBOT02` [actor](https://github.com/caiks/TBOT02/blob/master/actor.h) node is a dynamic version of the `TBOT01` [actor](https://github.com/caiks/TBOT01/blob/master/actor.h) node. 

TODO

It is given a *model*, a goal room and a mode of deciding actions. At each potential action it *applies* the *model* to the current *event* to determine its *slice*. The *slice* of the given *history*, e.g. `data009`, is *reduced* to a *histogram* of the label *variables* `location`, `motor` and `room_next`. 

In the simplest mode, `mode001`, this label *histogram* is *multiplied* by a *unit histogram* that defines the desired `room_next` given the goal room and the *slice's* `location`. For example, if the goal is room 6 and the `location` is room 1 then the `room_next` is room 4, rather than rooms 2 or 3. The turtlebot guesses `location` and then repeats the `motor` actions that tended in the past to lead to the desired goal. That is, the requested action is chosen at random according to the *probability histogram* implied by the *normalised reduction* to `motor`.


