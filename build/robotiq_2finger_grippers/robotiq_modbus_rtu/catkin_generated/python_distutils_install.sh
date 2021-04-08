#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/joe/vrep_ws/src/robotiq_2finger_grippers/robotiq_modbus_rtu"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/joe/vrep_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/joe/vrep_ws/install/lib/python3/dist-packages:/home/joe/vrep_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/joe/vrep_ws/build" \
    "/usr/bin/python3" \
    "/home/joe/vrep_ws/src/robotiq_2finger_grippers/robotiq_modbus_rtu/setup.py" \
     \
    build --build-base "/home/joe/vrep_ws/build/robotiq_2finger_grippers/robotiq_modbus_rtu" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/joe/vrep_ws/install" --install-scripts="/home/joe/vrep_ws/install/bin"
