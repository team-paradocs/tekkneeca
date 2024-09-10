FRI Vendor
==========
This ``package`` integrates KUKA's Fast Robot Interface (FRI) into the ROS 2 ``ament_cmake`` build system. It leaves all source code untouched.

Write your own ROS 2 Application
--------------------------------
.. note::
    If you intend to use the robot quickly, you can use the ``lbr_fri_ros2_stack`` (:ref:`LBR FRI ROS 2 Stack`), which is built on top of this package. 

If you intend to use the FRI in your own ROS 2 project, you can do so through the following:

- Add this repository to your workspace

.. code-block:: bash

    git clone https://github.com/KCL-BMEIS/fri_vendor.git -b ros2

- In your ``package.xml`` add: 

.. code-block:: xml
    
    <depend>fri_vendor</depend>

- In your ``CMakeLists.txt`` add:

.. code-block:: cmake
    
    find_package(fri_vendor REQUIRED)
    find_package(FRIClient REQUIRED) # provided by fri_vendor

    # link to target
    ament_target_dependencies(<your target> fri_vendor)
    target_link_libraries(<your target> FRIClient::FRIClient)

    # for down-stream packages
    ament_export_dependencies(fri_vendor FRIClient)

And replace ``<your target>`` with the name of your target.

- In the build step, specify the desired FRI version

.. code-block:: bash

    colcon build --cmake-args -DFRI_CLIENT_VERSION=1.15

API
---
For the ``Doxygen`` generated API, checkout `fri <../../docs/doxygen/fri/html/hierarchy.html>`_.
