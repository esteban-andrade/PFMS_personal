# Quiz 8

## Part A

You have been provided piece of code your colleauges have developed for Assignment 1/2, and we will be debugging these pieces of code.

![Debugging Pain](https://media.giphy.com/media/6yRVg0HWzgS88/giphy.gif)

For questions 1-3 use [vector_ops.cpp](./a/vector_ops.cpp). The intent of this code was to create a STL container resembling a matrix of elements (2x5).

The code compiles (no compile-time error) and also runs (no run-time errors).  
Yet, it fails to display the values (you would anticipate to see two rows of values).

```
0 1 2 3 4
0 1 2 3 4
```

The code has a few points of failure causing unintended behaviour.

For questions 4-5 use [main.cpp](./a/main.cpp) and the [Shape](./a/shape.h), [Rectangle](./a/rectangle.h) and [RectangleHelper](./a/rectanglehelper.h) class.

The code compiles (no compile-time error) and also runs (no run-time errors). Yet, it fails to determine line intersects.
Further, the RectangleHelper class that was supposed to print properties seems to be not working either.

1. QUESTION: When you request a change in capacity [reserve keyword](http://www.cplusplus.com/reference/vector/vector/reserve/), does that create elements of the vector.
   No, this will only request that the vector capacity will have enough space to contain n elements. This this will only allow to ensure that the vector will have the size as specified in the reserve.

2) QUESTION: What line of code is the first point of failure that could have been detected if the STL container was accessed correctly.
   the for loop to create and store data is not properly constructed. it has to be neccesary to make a for loop and inside this another for loop that will start pushing the elements of a another vector into the bigger matrix vector. Also the big vector of matrix it is made of doubles. And and it tried to store integer data into a double stl container. THis is not compatible. moreover, it was neccessary to store the elements in a temporary vector inside the for loop that will get populated based on the requirements and push back this vector into the other big vector.

3. QUESTION: Once errors associated with creating / storing data into the matrix are fixed, what additional error in this code results in incorrect matrix size.
   the issue with the correct matrix size is that the is it reserved the size to have 5 elements and it is strying to storee 6 elements (idx <= cols). Also it was trying to pass different data type into the vector of doubles.

4. QUESTION: Why does the intercept method in `Rectangle` fail to report the intercept correctly?
   it is clear that the offset is not not working. the rectangle does not offsets the centre x and centre y. The reason of this is because the object rect in not being pass by reference to be analysed. Also there are two member variables called centre_x and centre_y one in shape and one in rectangle. Thus is unclear to which value this is being applied

5) QUESTION: Why does the printArea method in `RectangleHelper` not print the correct area?
   This is because is a new instance of an object. In order to get the get the area of the created object, this has to be pass to this particular function in order to be calculated

## Part B

A package `topics_masterclass` is provided in the [tutorials/week10/starter directory](../../tutorials/week10/starter),link it to your workspace.
To copy:

```bash
cd ~/catkin_ws/src/
ln -s <path-to-git>/tutorials/week10/starter/topics_masterclass
```

Build the package using the `catkin_make` command (what folder do you need to be in to execute this command?).

You will also need the stage_ros package, install it by below (replace kinetic with melocdic if you have 18.04 and ROS melodic)

```bash
sudo apt-get install ros-kinetic-stage-ros
```

Start roscore in one terminal

```bash
roscore
```

Start stage_ros in another terminal (replace kinetic with melocdic if you have 18.04 and ROS melodic)

```bash
rosrun stage_ros stageros /opt/ros/kinetic/share/stage/worlds/simple.world
```

You should have the simulator appear
![Simple World in Stage Simulator](http://4.bp.blogspot.com/_B6REL4AVpFA/Szk9ipweWTI/AAAAAAAAALc/orflaXzpcZk/s400/Picture+2.png)

The goal is to modify the provided package to obtain pose of robot (x,y yaw) from nav_msgs/Odometry.

1. QUESTION: How would you access orientation in the `nav_msgs/Odometry` message (HINT: rosmsg show). The answer needs full path to orinettaion from the msg (for String we had msg.data)
   it can be accessed by getting the information from geometry_msgs Quaternion orientation, where we can access float from x,y,z,w in order to get the orientation.
   it will be msg->pose.pose.orientation and from this we can access either x,y,z,w depending on the needs

2) QUESTION: What type of message is the orientation?
   the type of message is a geometry_msgs/Quaternion message type and inside of this x,y,z,w are float64

3) QUESTION: Where is time of this message stored?
   the time is stored under the header as a time stamp under header

4) TASK: Use the [ROS TF] library helper function to get yaw from the orientation and print to screen (standard out)
   the tf::Quaterninion will get the obtain a quaternion from the tf, A matrix wil get created and the yaw can be calculated using getRPY
   tf::Quaternion q(
   msg->pose.pose.orientation.x,
   msg->pose.pose.orientation.y,
   msg->pose.pose.orientation.z,
   msg->pose.pose.orientation.w);
   tf::Matrix3x3 m(q);
   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);
   std::cout<<yaw<<std::endl

5) TASK: Print the pose of robot (x,y yaw) to screen using ROS_INFO_STREAM

double x = msg->pose.pose.position.x;
double y = msg->pose.pose.position.y;
ROS_INFO_STREAM(" x: " << x << " y: " << y << " yaw :" << yaw);

tf::Quaternion q(
msg->pose.pose.orientation.x,
msg->pose.pose.orientation.y,
msg->pose.pose.orientation.z,
msg->pose.pose.orientation.w);
tf::Matrix3x3 m(q);
double roll, pitch, yaw;
m.getRPY(roll, pitch, yaw);
double x = msg->pose.pose.position.x;
double y = msg->pose.pose.position.y;
//Ros equivalent to std::cout is a ROS_INFO_STREAM
ROS_INFO_STREAM(" x: " << x << " y: " << y << " yaw :" << yaw);

[ros tf]: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
