#include "mapcore.h"

mapcore m_core(24.031111, 49.842957, 8);

cdirprovider m_prov("D:\\DRON ECOTEST\\tile\\");

mapdraw m_draw(&m_core,&m_prov,0);


void mapcore::map_xy2longlatt(u_long x,u_long y,double &longt,double &latt)
{
    double t_x=x;
    double t_y=y;
    longt=360 * (t_x / (256 * (1 << zoom))) - 180;
    double term = 0.5 - t_y / (256 * (1<<zoom));
    double exponent = term * 4 * M_PI;
    double numerator = exp(exponent) - 1;
    double denominator = exp(exponent) + 1;
    latt = asin(numerator / denominator)/M_PI*180;
}
void mapcore::screen_xy2longlatt(u_long x,u_long y,double &longt,double &latt)
{
    double t_x=x+offs_map.x;
    double t_y=y+offs_map.y;
    longt=360 * (t_x / (256 * (1 << zoom))) - 180;
    double term = 0.5 - t_y / (256 * (1<<zoom));
    double exponent = term * 4 * M_PI;
    double numerator = exp(exponent) - 1;
    double denominator = exp(exponent) + 1;
    latt = asin(numerator / denominator)/M_PI*180;
}

void mapcore::longlatt2map_xy(u_long &x,u_long &y,smappoint coord)
{
    double sinLatitude = sin(coord.latt * M_PI/180);
    x = ((coord.longt + 180) / 360) * 256 * (1<<zoom);
    y = (0.5-log((1 + sinLatitude) / (1-sinLatitude)) / (4 * M_PI)) * 256 * (1<<zoom);
}

void mapcore::longlatt2screen_xy(u_long &x,u_long &y,smappoint coord)
{
    longlatt2map_xy(x,y,coord);
    x-=offs_map.x;
    y-=offs_map.y;
}

void mapcore::gettiles(spoint&how)
{
    longlatt2map_xy(offs_map.x,offs_map.y,m_view);
    offs_map.x=offs_map.x-(screen_size.x>>1);
    offs_map.y=offs_map.y-(screen_size.y>>1);
    how.x=(screen_size.x>>8)+1;
    how.y=(screen_size.y>>8)+1;
}

void mapcore::setcreensize(int x,int y)
{
    screen_size.x=x;
    screen_size.y=y;
}


void mapcore::scroll_h(int how)
{
    u_long x,y;
    longlatt2map_xy(x,y,m_view);
    x+=how;
    map_xy2longlatt(x,y,m_view.longt,m_view.latt);
}

void mapcore::scroll_v(int how)
{
    u_long x,y;
    longlatt2map_xy(x,y,m_view);
    y+=how;
    map_xy2longlatt(x,y,m_view.longt,m_view.latt);
}

void mapdraw::draw(QPainter &p)
{
    if(!p_view)
        return;
    p_core->setcreensize(p_view->contentsRect().width(),p_view->contentsRect().height());
    spoint how;
    p_core->gettiles(how);
    int bx=(p_core->offs_map.x>>8);
    int by=(p_core->offs_map.y>>8);
    int zoom = p_core->getzoom();
    QImage image;
    for(int i=0;i<how.x+1;i++)
    {
        for(int j=0;j<how.y+1;j++)
        {
            if(p_prov->gettile(zoom,bx+i,by+j,image))
            p.drawImage(i*256,j*256,image);
        }
    }
}
