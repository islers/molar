#ifndef CVMATDISPLAY_H
#define CVMATDISPLAY_H

#include <QQuickItem>
#include <QImage>
#include <QQuickImageProvider>
#include <QMutex>

#include "SceneHandler.h"


class CVMatDisplay : public QQuickItem, public QQuickImageProvider
{
    Q_OBJECT

public:
	CVMatDisplay();
	~CVMatDisplay();
	
    //Q_PROPERTY(QString name READ name WRITE setName)
    //Q_PROPERTY(QColor color READ color WRITE setColor)

public:
	QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize);
	void updateImage( cv::Mat _newImage );
	void updateOriginal( cv::Mat _newImage );
	void updateThresholdImage( cv::Mat _newImage );
	void updatePreProcessImage( cv::Mat _newImage );
	void updatePreThresholdImage( cv::Mat _newImage );

	// called after - surprise - all updates are done: emits a signal for the qml objects
	void updatesDone();

	void convertImage( cv::Mat& _cvImage, cv::Mat& _cvTemp, QImage& _qImage );

signals:
	void imageUpdated();
    /*CVMatDisplay(QQuickItem *parent = 0);

    QString name() const;
    void setName(const QString &name);

    QColor color() const;
    void setColor(const QColor &color);*/

private:

	QMutex pLock;
	
    QImage _qimage;
    cv::Mat _tmp;

    QImage _qOriginal;
    cv::Mat _tOriginal;

    QImage _qThresholdImage;
    cv::Mat _tThresholdImage;

    QImage _qPreProcessImage;
    cv::Mat _tPreProcessImage;

    QImage _qPreThresholdImage;
    cv::Mat _tPreThresholdImage;
};

#endif // CVMATDISPLAY_H
