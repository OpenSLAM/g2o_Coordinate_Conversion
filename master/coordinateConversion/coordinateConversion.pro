QT += core
QT -= gui

CONFIG += c++11
QMAKE_CXXFLAGS += -std=c++0x
TARGET = coordinateConversion
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    g2o_types.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
INCLUDEPATH += /home/fyj/ThirdParty/OPENCV2411/include \
               /home/fyj/ThirdParty/OPENCV2411/include/opencv \
               /home/fyj/ThirdParty/OPENCV2411/include/opencv2 \
               /usr/include/eigen3 \
               /usr/include/suitesparse


LIBS += \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_calib3d.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_contrib.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_core.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_features2d.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_flann.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_gpu.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_highgui.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_imgproc.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_legacy.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_ml.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_nonfree.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_objdetect.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_ocl.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_photo.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_stitching.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_superres.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_ts.a \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_video.so \
/home/fyj/ThirdParty/OPENCV249/lib/libopencv_videostab.so \
/usr/lib/x86_64-linux-gnu/libXext.so \
/usr/lib/x86_64-linux-gnu/libX11.so \
/usr/lib/x86_64-linux-gnu/libICE.so \
/usr/lib/x86_64-linux-gnu/libSM.so \
/usr/lib/x86_64-linux-gnu/libGL.so \
/usr/lib/x86_64-linux-gnu/libGLU.so \
/usr/local/lib/libg2o_cli.so \
/usr/local/lib/libg2o_core.so \
/usr/local/lib/libg2o_csparse_extension.so \
/usr/local/lib/libg2o_ext_csparse.so \
/usr/local/lib/libg2o_ext_freeglut_minimal.so \
/usr/local/lib/libg2o_incremental.so \
/usr/local/lib/libg2o_interactive.so \
/usr/local/lib/libg2o_interface.so \
/usr/local/lib/libg2o_opengl_helper.so \
/usr/local/lib/libg2o_parser.so \
/usr/local/lib/libg2o_simulator.so \
/usr/local/lib/libg2o_solver_cholmod.so \
/usr/local/lib/libg2o_solver_csparse.so \
/usr/local/lib/libg2o_solver_dense.so \
/usr/local/lib/libg2o_solver_eigen.so \
/usr/local/lib/libg2o_solver_pcg.so \
/usr/local/lib/libg2o_solver_slam2d_linear.so \
/usr/local/lib/libg2o_solver_structure_only.so \
/usr/local/lib/libg2o_stuff.so \
/usr/local/lib/libg2o_types_data.so \
/usr/local/lib/libg2o_types_icp.so \
/usr/local/lib/libg2o_types_sba.so \
/usr/local/lib/libg2o_types_sclam2d.so \
/usr/local/lib/libg2o_types_sim3.so \
/usr/local/lib/libg2o_types_slam2d.so \
/usr/local/lib/libg2o_types_slam2d_addons.so \
/usr/local/lib/libg2o_types_slam3d.so \
/usr/local/lib/libg2o_types_slam3d_addons.so \
/usr/lib/x86_64-linux-gnu/libcholmod.so \
-lcufft -lnpps -lnppi -lnppc -lcudart -lrt -lpthread -lm -ldl

HEADERS += \
    g2o_types.h
