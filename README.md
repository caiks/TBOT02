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

[Models of the scan substrate](#Models)

[Models conditioned on location and position](#Models_conditioned)

[Location and position observer](#Observer)

[Modelling with an unbiased controller](#Unbiased)

[Timewise frames](#Timewise)

[Motor actions](#Motor)

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

We can see that for the *2-level model* there is a jump in *likelihood* between 1,000 records and 2,000 records. There is another jump later on between 20,000 records and 100,000 records. Unlike the *1-level model* the *likelihood*  of the *2-level model* generally increases with *size*, at least at the root *fud*. The reason for this is probably because the *1-level-model* was *modelled* with non-sequential *history* - each  *event* consisted of 60 *scan variables* taken at random from the 360 of the *substrate*. The *2-level-model* is *modelled* with sequential *history*, so it is only after around 2,000 records or 8 minutes that all of the rooms havee been visited. Although *active modelling* with small thresholds of a few hundred *events* may produce a root *fud* with a low *alignment*, that is not to say that a large *model* with many nodes, though lopsided, might not have high *likelihood* nonetheless. In general, however, a large initial threshold looks desirable.

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
We can see that at the 7000th *event* the induce threshold is crossed and the root *fud* is *modelled*, having a *diagonal* of `40.7`. The *model* 43 eventually has 36 *fuds*. The last *fud* only has a diagonal of `4.3`. Note that there is a very large *slice* of 12,204 *events* which has no *alignment* - presumably because the turtlebot is looking into empty space up to the range of the lidar.

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

Let us simulate the dynamic *modelling* of *model* 27. We will begin with a threshold *slice size* of 1000 and terminate when we obtain 127 *fuds*.

