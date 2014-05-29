ZgeBullet Demo Project #2
=========================

This is a ZGameEditor project which demonstrates usage of dynamic heightfield terrains. Press and hold left mouse button to change the terrain on Windows (single touch on Android). Press Space to remove all spawn spheres on Windows (touch with more than one finger on Android).

To run the demo on Windows, copy the ZgeBullet.dll to the same directory as this project and/or the generated EXE file.

To run the demo application on Android, activate the menu item "Android: Build APK (debug)" in ZGE. Then, copy the appropriate libgeEBullet.so file to the libs\ermabi\ folder of the generated directory and regenerate the Android application again. In the bin\ folder of the generated directory you will find the .apk file which you install on your Android device.

Note: Use the libZgeBullet.so file compatible with your device. In other case, the application can crash. It is recommended to use modern (e.g., ARMv7-A compatible) devices which can achieve much better performance for large amount of objects on a scene. If you have older device, you can, for instance, decrease the number of simulated physical objects to achieve smoother movements.

Have a fun!
Rado1.
