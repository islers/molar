TEMPLATE = app
#TARGET = molar

QT += core multimedia script declarative opengl widgets gui qml quick multimediawidgets
QT -= android

unix:!mac {
    LIBS += -lopencv_core \
        -lopencv_highgui \
        -lopencv_imgproc \
        -lopencv_video \
        -lopencv_features2d \
        -lopencv_ml \
        -lboost_system \
        -lboost_filesystem
}

INCLUDEPATH += ../code_base/include/core \
               ../code_base/include/dynamic_modules \
               ../code_base/include/filters \
               ../gui/include \
               ../code_base/stis/include \
               ../code_base/stis/ticpp/include

DESTDIR =../run

include(molar.pri)



OTHER_FILES += \
    ../gui/rsc/qml/AlgorithmChooser.qml \
    ../gui/rsc/qml/AlgorithmListModel.qml \
    ../gui/rsc/qml/AlgorithmSetting.qml \
    ../gui/rsc/qml/ClassificationContent.qml \
    ../gui/rsc/qml/main.qml \
    ../gui/rsc/qml/ModarSwitchButton.qml \
    ../gui/rsc/qml/ModarButton.qml \
    ../gui/rsc/qml/MolarSettings.qml \
    ../gui/rsc/qml/NewFeatureSet.qml \
    ../gui/rsc/qml/NewObjectType.qml \
    ../gui/rsc/qml/NewStreamWindow.qml \
    ../gui/rsc/qml/SceneControlContent.qml \
    ../gui/rsc/qml/TypeChooser.qml \
    ../gui/rsc/qml/WhiteButton.qml

RESOURCES += \
    rsc/resource.qrc
