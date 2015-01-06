molar
=====
overview
-----
MOLAR ("Multiple Object Localization And Recognition") is a C++ framework to detect, track and recognize multiple simple objects in simple scenes in realtime. Each detected object is instantiated as a c++ object of the type recognized. It was written to detect microrobots in camera streams of cameras mounted on microscopes and distinguish them from other objects in the scene. For the kinematics of the different object types several models have been coded using different filter types (e.g. Extended Kalman filters) which can be extended easily. Recognition of types is based on FAST/BRISK descriptors and a Boost classifier. A QML-based graphical user interface is provided that gives easy access to the functionality of the code and also allows to specify new object types at runtime. The code is OpenCV based and has been run both on Linux and Windows.  
  
Here is an example with tracked objects in an image stream:
![example output](http://stewess.github.io/molar/img/helix_example_output.png)

repository roadmap
----
The subfolders contain their own instructions, but here are some directions:  
  
<b>code_base:</b> all the code needed for tracking and recognition  
**documentation:** some information  
**examples:** an example for a GUI configuration and an example on using MOLAR without the GUI  
**gui:** the code for the GUI along with QtCreator project files to ease compilation  
**run:** basic folder structure for MOLAR to get started without issues

requirements & dependencies
----
To run MOLAR you need:
- OpenCV (The code was written for v2.4.9) [official website](http://opencv.org/)
- Boost system & filesystem libraries [official website](http://www.boost.org/)

If you'd like to use the graphical user interface, you'll additionally need:
- Qt (the code is tested with v5.4) [official website](http://qt-project.org/) (Get Downloads>>Community. You might have to list different download options if you're on a 32bit computer. Default settings for the installer should be fine.)
