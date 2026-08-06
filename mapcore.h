#ifndef MAPCORE_H
#define MAPCORE_H
#define _USE_MATH_DEFINES
#include <cmath>
#include <string.h>
#include<QWidget>
#include<QPainter>
#include<vector>
#include<algorithm>
using namespace std;
#define TILE_SIZE	        256
#define EARTH_RADIUS        6378137.0
#define ORIGIN_SHIFT        (2.0 * M_PI * EARTH_RADIUS / 2.0)
#define INITIAL_RESOLUTION  (2.0 * M_PI * EARTH_RADIUS / TILE_SIZE)

#define MARKER_RED        1
#define MARKER_BLUE       2
#define M_CHEMISTRY       3
#define M_RADIATION       4
#define M_RECT            5
#define M_ELLIPSE         6
#define M_POLYGON         7


typedef struct tsmappoint
{
    double longt;
    double latt;
}smappoint;

typedef   long long  int u_long;


typedef struct tspoint
{
    u_long x;
    u_long y;
}spoint;


typedef struct tsmarker
{
    tsmappoint pt;
    int type;
}smarker;

extern vector<smarker> dangers;

extern vector<smappoint>dronepath;

inline bool operator ==(const smappoint &p0,const smappoint &p1)
{
    return((p0.latt==p1.latt)&&(p0.longt==p1.longt));
}

typedef   long long  int u_long;
class mapdraw;
class ViewerMap;
class mapcore
{
    friend  mapdraw;
    friend ViewerMap;
    int zoom;
    spoint screen_size;
    smappoint m_view;
    spoint offs_map;
    char path[1024];
public:
    mapcore(tsmappoint v,int z)
    {
        zoom=z;
        m_view=v;
        offs_map={0};
    }
    mapcore(double longt,double latt,int z)
    {
        m_view.longt=longt;
        m_view.latt=latt;
        zoom=z;
        offs_map={0};
        offs_map={0};
    }
    void setcreensize(int x,int y);
    inline void setpath(const char *ps)
    {
        strcpy_s(path,ps);
    }
    inline char* getpath()
    {
        return path;
    }
    inline int getzoom()
    {
        return zoom;
    }
    inline void setzoom(int z)
    {
        zoom=z;
    }
    inline smappoint getview()
    {
        return m_view;
    }
    inline void setview(smappoint v)
    {
        m_view=v;
    }
    void gettiles(spoint &how);
    void map_xy2longlatt(u_long x,u_long y,double &longt,double &latt);
    void screen_xy2longlatt(u_long x,u_long y,double &longt,double &latt);
    void longlatt2map_xy(u_long &x,u_long &y,smappoint coord);
    void longlatt2screen_xy(u_long &x,u_long &y,smappoint coord);
    void scroll_h(int how);
    void scroll_v(int how);
};
extern mapcore m_core;

class cdirprovider
{
    char path[1024];
public:
    cdirprovider(const char *p)
    {
        strcpy_s(path,p);
    }
    void setpath(const char *p)
    {
        strcpy_s(path,p);
    };
    bool gettile(int z,int x,int y,QImage &img)
    {
        if(path[0]==0)
            return false;
        char t[4096];
        sprintf(t,"%s%d\\%d\\%d.png",path,z,x,y);
        return img.load(t);
    }


};

class mapdraw
{
    QWidget *p_view;
    cdirprovider *p_prov;
    mapcore *p_core;
public:
    mapdraw(mapcore *p_c,cdirprovider *p_d,QWidget *p_v=0)
    {
        p_view=p_v;
        p_prov=p_d;
        p_core=p_c;
    }
    mapdraw(mapcore *p_c)
    {
        p_core=p_c;
        p_view=0;
        p_prov=0;
    }
    inline void setprovider(cdirprovider *p_d)
    {
        p_prov=p_d;
    }
    inline void setwindow(QWidget *p_v)
    {
        p_view=p_v;
    }
    void draw(QPainter &p);
};
extern mapcore m_core;
extern cdirprovider m_prov;
extern mapdraw m_draw;
extern vector<smarker> dangers;
#endif // MAPCORE_H

