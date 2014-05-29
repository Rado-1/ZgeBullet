ZgeBullet Demo Project #1
=========================

This is a ZGameEditor project file which demonstrates the basic possibilities of the ZgeBullet library and shows its usage. In particular, you can see how to create 3D physical world, define collision shapes, place rigid bodies to a physical world, bind graphical objects to their rigid bodies, step simulation time, and update properties of objects by discrete simulation.

The demo uses a generic framework of creating 3D objects, specification their properties, and placing them to a world. Users are, therefore, allowed to easily define their own object formations and to modify or extend this demo. See the 'InitScene' component for details on how to define formations of objects in scenes.

To run the demo on Windows, copy the ZgeBullet DLL to the same directory as this project and/or the generated EXE file.

To run the demo application on Android, activate the menu item "Android: Build APK (debug)" in ZGE. Then, copy the appropriate libZgeBullet.so file to the libs\ermabi\ folder of the generated directory and regenerate the Android application again. In the bin\ folder of the generated directory you will find the .apk file which you install on your Android device.

Note: Use the libZGEBullet.so file compatible with your device. In other case, the application can crash. It is recommended to use modern (e.g., ARMv7-A compatible) devices which can achieve much better performance for large amount of objects on a scene. If you have older device, you can, for instance, decrease the number of simulated physical objects to achieve smoother movements.

Have a fun!
Rado1.
