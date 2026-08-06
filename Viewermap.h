#ifndef VIEWERMAP_H
#define VIEWERMAP_H

#include <QObject>
#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <QMessageBox>
#include <QPixmap>
#include "mapcore.h"

#define MAX_ZOOM 14

class ViewerMap : public QWidget
{
    Q_OBJECT

    QPoint start_coords;
    QPixmap *p_map;
    QPoint mpos;
    tsmappoint m_countur[2]={{-1.0,-1.0},{-1,-1}};
public:
    smappoint countur[2]={{-1,-1},{-1,-1}};
    explicit ViewerMap(QWidget *parent = nullptr);
    virtual ~ViewerMap()
    {
        if(p_map)
            delete p_map;
    }
    void setcontur(tsmappoint pt0,tsmappoint pt1);
    void setdronepos(tsmappoint pt,int danger_type);
protected:


    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;



    void wheelEvent(QWheelEvent* event) Q_DECL_OVERRIDE;
    void mouseDoubleClickEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event)Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event)Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *event)Q_DECL_OVERRIDE;


signals:
};

#endif // VIEWERMAP_H
