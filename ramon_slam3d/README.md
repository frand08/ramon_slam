# 3DSLAM

## Install PCL library
    $ sudo apt install libpcl-dev

## Install Openni2_launch
    $ sudo apt-get install ros-noetic-openni2-launch 

## Install Octomap
    $ sudo apt-get install ros-noetic-octomap

## Install Open3D
    $ sudo apt-get install python3-pip
    $ pip install pybind11
    $ git clone --recursive https://github.com/intel-isl/Open3D
    $ cd Open3D
    $ ./util/install_deps_ubuntu.sh
    $ mkdir build
    $ cd build
    $ sudo cmake DBUILD_PYBIND11=OFF DBUILD_PYTHON_MODULE=OFF ..
    $ make install

### If "undefined reference to `__atan2f_finite'" (eg, https://github.com/intel-isl/Open3D/issues/1909)
- Install filament dependences (https://github.com/google/filament/blob/main/BUILDING.md#linux)
    
    - clang-7 or higher
    - libglu1-mesa-dev
    - libc++-7-dev (libcxx-devel and libcxx-static on Fedora) or higher
    - libc++abi-7-dev (libcxxabi-static on Fedora) or higher
    - ninja-build
    - libxi-dev

- As we need python for Google Filament installation, create a Python alias in case you dont have it, eg., for Python 3.8

        $ alias python=python3.8

- Install Google Filament

        $ sudo apt-get install clang
        $ git clone https://github.com/google/filament.git
        $ cd filament
        $ env CC=/usr/bin/clang CXX=/usr/bin/clang++ FILAMENT_REQUIRES_CXXABI=true ./build.sh -c release
    - NOTE: If "/usr/bin/env: ‘python’: No such file or directory" install
        
          $ sudo apt-get install python-is-python3


- Check https://github.com/intel-isl/Open3D/issues/1909