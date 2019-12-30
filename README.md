# ROS2 demo application sending fixed size images

This is the demo code show cased at ROSCon 2019 featuring [rmw_iceoryx](https://github.com/ros2/rmw_iceoryx) and its zero-copy capabilities.

## Usage

In order to leverage the full potential of zero-copy, we assume in the following that you use `rmw_iceoryx_cpp` as the underlying RMW implementation.
That is, prefixing the commands below by `RMW_IMPLEMENTATION=rmw_iceoryx_cpp` or genereally setting the environment variable in its respective terminal.

The application consists of two executables:
* image_transport_publisher
* image_transport_subscriber

Both executables have to be started with a specific image size to agree on a common topic.

### Start the publisher

```
ros2 run fixed_size_image_transport image_transport_publisher
```

This sends the default values - VGA image size with 15 Hz.

### Start the subscriber

```
ros2 run fixed_size_image_transport image_transport_subscriber
```

Like the publisher, this subscribes to default values of VGA.

### Modifying the data volume, frequency and transport method

The publisher/subscriber tuple can be modified to send/receive different image sizes with various frequencies and either with loaned messages or classic stack based message allocation.

For this, start the publisher with additional command line arguments as such:
```
ros2 run fixed_size_image_transport image_transport_publisher <frequency> <image size> <transport method>
```
The subscriber can then be started as such:
```
ros2 run fixed_size_image_transport image_transport_subscriber <image_size> <--no-gui>
```
The `--no-gui` arguments becomes handy when sending large image sizes to not display them with each callback.
