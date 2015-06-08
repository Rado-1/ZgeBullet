ZgeBullet Demo Project #3
=========================

This is a ZGameEditor project file which demonstrates the basic possibilities
of the ZgeBullet library to create and use Raycast Vehicle, a simple car
simulation.

To drive the car: move mouse (or touch on Android) left/right to steer, put
mouse to upper section to move forward, put mouse to lower section to break.
Press right mouse button (or double-touch on Android) to reinitialize the scene.

To run the demo on Windows, copy the ZgeBullet DLL to the same directory as this
project and/or the generated EXE file.

To run the demo application on Android, activate the menu item
"Android: Build APK (debug)" in ZGE. Then, copy the appropriate libZgeBullet.so
file to the libs\ermabi\ folder of the generated directory and regenerate the
Android application again. Tthe bin\ folder of the generated directory will
contain the .apk file which you install on your Android device.

Note: Use the libZGEBullet.so file compatible with your device. In other case,
the application can crash. It is recommended to use modern (e.g., ARMv7-A
compatible) devices which can achieve much better performance for large amount
of objects on a scene. If you have older device, you can, for instance, decrease
the number of simulated physical objects to achieve smoother movements.

Have fun!
Rado1.
