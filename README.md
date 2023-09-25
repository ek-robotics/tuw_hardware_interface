# TUW Hardware Interface
This package contains a template to operate hardware with the hardware interface of the [`ros_control`](https://wiki.ros.org/ros_control) package.
The template has specific implementations for the [Dynamixel MX-106](https://emanual.robotis.com/docs/en/dxl/mx/mx-106-2/) servo motor and the [Trinamic TCMC-1640](https://www.trinamic.com/products/modules/details/tmcm-1640/) brushless motor controller board.
The template is designed to define setup as well as hardware specific properties in custom `yaml` files.
Additionally, a reconfigure server is hosted for the hardware in order to change hardware properties during operation.

Note: some hardware properties might not be changeable during operation or may require the hardware to be set to a specific state before the hardware parameters can be changed.

Make sure to understand how `ros_control` works before making any changes to this package.
A good starting point to learn about `ros_control` is [this](https://fjp.at/posts/ros/ros-control/) post.
Furthermore, make sure to understand the functionality of dynamic reconfigure and [`ddynamic_reconfigure`](https://github.com/pal-robotics/ddynamic_reconfigure).

## `tuw_hardware_interface` - Metapackage
Metapackage for `tuw_hardware_interface`.

## `tuw_hardware_interface_dynamixel`
This package contains the hardware interface specific to the requirements of the Dynamixel MX-106.
For this reason this package is based on the `tuw_hardware_interface_template`.
This package is exported as a library in order to be used by other packages.
For more details check  [`tuw_hardware_interface_dynamixel/README.md`](./tuw_hardware_interface_dynamixel/README.md).

## `tuw_hardware_interface_template`
This package contains a template version of the hardware interface with the aforementioned features.
For more details check  [`tuw_hardware_interface_template/README.md`](./tuw_hardware_interface_template/README.md).

## `tuw_hardware_interface_trinamix`
This package contains the hardware interface specific to the requirements of the Trinamic TCMC-1640.
For this reason this package is based on the `tuw_hardware_interface_template`.
This package is exported as a library in order to be used by other packages.
For more details check  [`tuw_hardware_interface_trinamic/README.md`](./tuw_hardware_interface_trinamic/README.md).
