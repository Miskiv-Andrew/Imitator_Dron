#include "Viewermap.h"

#include "mapcore.h"

#include <qevent.h>

extern mapdraw m_draw;
extern mapcore m_core;

QImage marker;
vector<tsmappoint>tmp_p;

vector<smappoint>dronepath;

smappoint dronepos={0,0};
QImage dron,radiat,chem;
vector<smarker> dangers;

void drawdronepath(QPainter &p)
{
    p.setPen(QPen(Qt::red, 1, Qt::SolidLine));
    for(int i=0;i<(int)dronepath.size()-1;i++)
    {
        u_long x0,y0,x1,y1;
        m_core.longlatt2screen_xy(x0,y0,dronepath[i]);
        m_core.longlatt2screen_xy(x1,y1,dronepath[i+1]);
        p.drawLine(x0,y0,x1,y1);
    }
}

void drawcountur(QPainter &p,smappoint pt[])
{
    if((pt[0].longt==-1)||(pt[0].latt==-1)||(pt[1].longt==-1)||(pt[1].latt==-1))
        return;
    p.setPen(QPen(Qt::blue, 2,Qt::SolidLine));
    u_long x0,y0,x1,y1;
    m_core.longlatt2screen_xy(x0,y0,pt[0]);
    m_core.longlatt2screen_xy(x1,y1,pt[1]);
    p.drawRect(QRect(x0,y0,x1-x0,y1-y0));
}

void drawdrone(QPainter &p)
{
    u_long x, y;
    m_core.longlatt2screen_xy(x,y,dronepos);
    p.setPen(QPen(Qt::black, 2, Qt::SolidLine));
    p.drawLine(x,y,x,y-16);
    p.drawImage(x-16,y-48 ,dron);
}

void drawdangers(QPainter &p)
{
    for(size_t i=0;i<dangers.size();i++)
    {
        u_long x,y;
        m_core.longlatt2screen_xy(x,y,dangers[i].pt);
        p.setPen(QPen(Qt::black, 2, Qt::SolidLine));
        p.drawLine(x,y,x,y-16);
        if(dangers[i].type==0x1)
            p.drawImage(x-16,y-48 ,radiat);
        else if(dangers[i].type==0x2)
            p.drawImage(x-16,y-48 ,chem);
        else
        {
            p.drawImage(x-16,y-64 ,chem);
            p.drawImage(x-16,y-48 ,radiat);
        }
    }
}

ViewerMap::ViewerMap(QWidget *parent)
    : QWidget{parent}
{
    marker.load("D:\\Timoshin_DRON_Kiev\\Imitator\\stage_4\\img\\reds32.png");

    dron.load("D:\\Timoshin_DRON_Kiev\\Imitator\\stage_4\\img\\dron32.png");

    dronepos=m_core.getview();

    radiat.load("D:\\Timoshin_DRON_Kiev\\Imitator\\stage_4\\img\\rad32.png");

    chem.load("D:\\Timoshin_DRON_Kiev\\Imitator\\stage_4\\img\\chem32.png");

    p_map=new QPixmap(4096,4096);
}

void ViewerMap::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter;
    painter.begin(this);
    QPainter p(p_map);
    m_draw.draw(p);
    int ox=m_core.offs_map.x%256;
    int oy=m_core.offs_map.y%256;

    QRect screen(0,0,m_core.screen_size.x,m_core.screen_size.y);
    QRect im(ox,oy,m_core.screen_size.x,m_core.screen_size.y);
    painter.drawPixmap(screen,*p_map,im);
    drawdronepath(painter);
    drawdrone(painter);//add this line
    drawdangers(painter);
    drawcountur(painter,m_countur);
    painter.end();
}


void ViewerMap::wheelEvent(QWheelEvent* event)
{
    int z=m_core.getzoom();
    if((event->angleDelta().y() > 0)&&(z < MAX_ZOOM)&&(event->angleDelta().y()>16)) // up Wheel
    {
        m_core.setzoom(++z);
        update();
    }
    else if((event->angleDelta().y() < 0)&&(z>0)&&(abs(event->angleDelta().y())>16))
    {
        m_core.setzoom(--z);
        update();
    }
}


void ViewerMap::mouseDoubleClickEvent(QMouseEvent *event)
{
/*     QPoint pt=event->pos();
     m_core.xy2longlatt(pt.x(),pt.y(),dronepos.longt,dronepos.latt);
     update();
*/
}

void ViewerMap::setcontur(tsmappoint pt0,tsmappoint pt1)
{
     m_countur[0]=pt0;
     m_countur[1]=pt1;
     m_core.setview(pt0);
     dronepos=pt0;
     update();

}

void ViewerMap::setdronepos(tsmappoint pt,int danger_type)
{
    // if(danger_type!=0)
    // {
    // smarker tmp;
    // tmp.pt=dronepos;
    // tmp.type=danger_type;
    // dangers.push_back((tmp));
    // }
    // dronepos=pt;
    // update();

    if(danger_type!=0)
    {
        smarker tmp;
        tmp.pt=dronepos;
        tmp.type=danger_type;
        dangers.push_back((tmp));
    }
    dronepath.push_back(pt);//add this line
    dronepos=pt;
    update();
}




void ViewerMap::mousePressEvent(QMouseEvent *event)
{
     if (event->button() == Qt::LeftButton)
     {
        mpos = event->pos();
        setCursor(Qt::PointingHandCursor);
     }
}

void ViewerMap::mouseMoveEvent(QMouseEvent *event)
{
     QPoint mousePos = event->pos();  // Получаем позицию мыши
     QPoint tmp=mousePos;
     if (event->buttons() & Qt::LeftButton)
     {
        mousePos -= mpos;
        mpos=tmp;
        if(abs(mousePos.x())>2)
            m_core.scroll_h(mousePos.x()*-1);
        if(abs(mousePos.y())>2)
            m_core.scroll_v(mousePos.y()*-1);
        repaint();
     }

}

void ViewerMap::mouseReleaseEvent(QMouseEvent *event)
{
     setCursor(Qt::ArrowCursor);
}
