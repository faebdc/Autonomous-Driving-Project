// Copyright @2018 Pony AI Inc. All rights reserved.

#include "perception/perception.h"

#include<cstdio>
#include<cmath>
#include<algorithm>
#include<vector>
using namespace std;

const int pic_num = 10;
const int lump_num = 300;
const int point_num = 150000;
const double inf = 1e9;
const int dir[4][2]={{-1,0},{0,-1},{1,0},{0,1}};

const double PER_LIMIT = 30;

const double block_size = 0.5;	//块大小
const int block_shift = lump_num/2;
//const double ground_block_size = 2.0;	//识别地面时块大小
const double ground_eps = 0.1;	//点高下限
const double sky_eps = 3;	//点高上限
const int pnum_limit = 20;	//块密度下限
const double union_dis = 1.2;	//并查集距离限制
//const int ground_point_eps = /**/0;	//地面点数下限
const double too_high_eps = 3;	//不能有太高的点
const double too_high_rate = 0.1;	//太高的点比例限制
const int building_total_eps = 5;	//不能有太多太高的点
const double Smooth_eps = 0.15;
const double car_area_least = 3.5;
const double car_area_largest = 80.0;
const double density_least = 10;	//car密度判断

struct Vector
{
	double x,y,z;
};
struct Pic
{
	Vector trans;
	Vector point[point_num];
	vector<Vector> v[point_num];
	int fa[point_num];
	int cc;
};

bool cmp(Vector a,Vector b)
{
	return (a.x<b.x);
}

Vector operator + (const Vector &a, const Vector &b)
{
	Vector resx;
	resx.x=a.x+b.x;
	resx.y=a.y+b.y;
	resx.z=a.z+b.z;
	return resx;
}

Vector operator - (const Vector &a, const Vector &b)
{
	Vector resx;
	resx.x=a.x-b.x;
	resx.y=a.y-b.y;
	resx.z=a.z-b.z;
	return resx;
}

Pic sxu[pic_num+1],*now;
double val[lump_num][lump_num],valx[lump_num][lump_num],sval[lump_num][lump_num],svalx[lump_num][lump_num];
int total[lump_num][lump_num];
int ground_total[lump_num][lump_num];
int building_total[lump_num][lump_num];
bool smooth[lump_num][lump_num];
bool out_smooth[lump_num][lump_num],smooth_border[lump_num][lump_num];
int pic_cc=0;
bool pic_cycle=0;

Vector getVector(Eigen::Vector3d t)
{
	Vector res;
	res.x=t.x();
	res.y=t.y();
	res.z=t.z();
	return res;
}

void GetData(Pic *now,const PointCloud& pc)
{
	now->trans=getVector(pc.translation);
	now->cc=0;
  
    for (int i = 0; i < pc.points.size(); i++) 
    {
        Eigen::Vector3d p = pc.points[i];
		p = pc.rotation * p;
		if(p.x()*p.x()+p.y()*p.y()<PER_LIMIT*PER_LIMIT)
		{
			now->point[now->cc]=getVector(p);
			now->cc++;
		}
	}
}


void DetectGround(Pic &xu, Pic &xux)
{
	int i,j;
	for(i=0;i<lump_num;i++)
	{
		for(j=0;j<lump_num;j++)
		{
			val[i][j]=inf;
			valx[i][j]=-inf;
		}
	}
	//for(i=0;i<N;i++)
	{
		for(j=0;j<xu.cc;j++)
		{
			Vector now=xu.point[j];
			int vx=floor(now.x/block_size);
			int vy=floor(now.y/block_size);
			vx+=block_shift;
			vy+=block_shift;
			val[vx][vy]=min(val[vx][vy],now.z);
			valx[vx][vy]=max(valx[vx][vy],now.z);
		}
	}
	for(i=0;i<lump_num;i++)
	{
		for(j=0;j<lump_num;j++)
		{
			sval[i][j]=inf;
			svalx[i][j]=-inf;
		}
	}
	for(i=0;i<pic_num;i++)
	{
		if(i>pic_cc && !pic_cycle)
			break;
		for(j=0;j<sxu[i].cc;j++)
		{
			Vector now=sxu[i].point[j];
			now=now+sxu[i].trans;
			now=now-xu.trans;
			int vx=floor(now.x/block_size);
			int vy=floor(now.y/block_size);
			vx+=block_shift;
			vy+=block_shift;
			sval[vx][vy]=min(sval[vx][vy],now.z);
			svalx[vx][vy]=max(svalx[vx][vy],now.z);
		}
	}






	{
		int px=0;
		for(j=0;j<xu.cc;j++)
		{
			Vector now=xu.point[j];
			int vx=floor(now.x/block_size);
			int vy=floor(now.y/block_size);
			vx+=block_shift;
			vy+=block_shift;
			if(sval[vx][vy]+ground_eps<now.z && now.z<sval[vx][vy]+sky_eps)
			//if(sval[vx][vy]+ground_eps<now.z)
			{
				xux.point[px]=xu.point[j];
				px++;
			}
		}
		xux.cc=px;
		xux.trans=xu.trans;
	}
}

void DfsSmooth(int a,int b)
{
	if(a<0 || a>=lump_num)
		return;
	if(b<0 || b>=lump_num)
		return;
	if(smooth[a][b])
		return;
	smooth[a][b]=1;
	int i,j;
	for(i=0;i<=3;i++)
	{
		int ta=a+dir[i][0];
		int tb=b+dir[i][1];
		if(abs(sval[ta][tb]-sval[a][b])<Smooth_eps)
			DfsSmooth(ta,tb);
	}
}

void Dfs_Out_Smooth(int a,int b)
{
	if(a<0 || a>=lump_num)
		return;
	if(b<0 || b>=lump_num)
		return;
	if(smooth_border[a][b])
		return;
	if(smooth[a][b])
		return;
	smooth_border[a][b]=1;
	int i,j;
	for(i=0;i<=3;i++)
	{
		int ta=a+dir[i][0];
		int tb=b+dir[i][1];
		DfsSmooth(ta,tb);
	}
}

void GetSmooth(Pic &xu)
{
	int i,j;
	DfsSmooth(block_shift,block_shift);
	Dfs_Out_Smooth(0,0);



	{
		int px=0;
		for(j=0;j<xu.cc;j++)
		{
			Vector now=xu.point[j];
			int vx=floor(now.x/block_size);
			int vy=floor(now.y/block_size);
			vx+=block_shift;
			vy+=block_shift;
			if(!smooth_border[vx][vy])
			{
				xu.point[px]=xu.point[j];
				px++;
			}
		}
		xu.cc=px;
	}
}



double CalcDis(Vector a,Vector b)
{
	double dx=(a.x-b.x);
	double dy=(a.y-b.y);
	double dz=(a.z-b.z);
	double res=sqrt(dx*dx+dy*dy+dz*dz);
	return res;
}

int getf(Pic &xu, int x)
{
	if(xu.fa[x]!=x)
		xu.fa[x]=getf(xu,xu.fa[x]);
	return xu.fa[x];
}

void Combine(Pic &xu, int a,int b)
{
	a=getf(xu,a);
	b=getf(xu,b);
	xu.fa[a]=b;
}

void GetUnion(Pic &xu)
{
	int i,j;
	sort(xu.point,xu.point+xu.cc,cmp);
	for(i=0;i<xu.cc;i++)
		xu.fa[i]=i;
	for(i=0;i<xu.cc;i++)
	{
		double y_up=inf,y_down=-inf;
		for(j=i+1;j<xu.cc;j++)
		{
			if(xu.point[j].x-xu.point[i].x>union_dis)
				break;
			if((xu.point[i].x+union_dis-xu.point[j].x)*(xu.point[i].x+union_dis-xu.point[j].x)
				+(xu.point[i].y-xu.point[j].y)*(xu.point[i].y-xu.point[j].y)<union_dis*union_dis)
			{
				if(xu.point[j].y>xu.point[i].y)
				{
					y_up=min(y_up,xu.point[j].y);
				}
				else
				{
					y_down=max(y_down,xu.point[j].y);
				}
			}
			if(CalcDis(xu.point[i],xu.point[j])<union_dis)
			{
				Combine(xu,i,j);
			}
			if(y_up<0.5*inf && y_down>-0.5*inf)
				break;
		}
	}
	for(i=0;i<xu.cc;i++)
		xu.v[i].clear();
	for(i=0;i<xu.cc;i++)
	{
		xu.v[getf(xu,i)].push_back(xu.point[i]);
	}
}


void DetectSingleObject(vector<Vector> &now,interface::perception::PerceptionObstacles &perception_result,Vector trans)
{
	double xa,xb,ya,yb;
	xa=ya=inf;
	xb=yb=-inf;
	int too_high_num=0;
	for(int j=now.size()-1;j>=0;j--)
	{
		xa=min(xa,now[j].x);
		xb=max(xb,now[j].x);
		ya=min(ya,now[j].y);
		yb=max(yb,now[j].y);
		int vx=floor(now[j].x/block_size);
		int vy=floor(now[j].y/block_size);
		vx+=block_shift;
		vy+=block_shift;
		if(now[j].z-sval[vx][vy]>too_high_eps)
			too_high_num++;
	}
	xa+=trans.x;
	xb+=trans.x;
	ya+=trans.y;
	yb+=trans.y;

	if(too_high_num>too_high_rate*now.size())
		return;


	if(now.size()/((xb-xa)*(yb-ya)) > density_least )
	if(car_area_largest>(xb-xa)*(yb-ya) && (xb-xa)*(yb-ya)>car_area_least)
	{
	    auto* obstacle = perception_result.add_obstacle();
	    obstacle->set_type(interface::perception::ObjectType::CAR);
	    {
	      auto* polygon_point = obstacle->add_polygon_point();
	      polygon_point->set_x(xa);
	      polygon_point->set_y(ya);
	      polygon_point->set_z(-8.18);
	    }
	    {
	      auto* polygon_point = obstacle->add_polygon_point();
	      polygon_point->set_x(xb);
	      polygon_point->set_y(ya);
	      polygon_point->set_z(-8.18);
	    }
	    {
	      auto* polygon_point = obstacle->add_polygon_point();
	      polygon_point->set_x(xb);
	      polygon_point->set_y(yb);
	      polygon_point->set_z(-8.18);
	    }
	    {
	      auto* polygon_point = obstacle->add_polygon_point();
	      polygon_point->set_x(xa);
	      polygon_point->set_y(yb);
	      polygon_point->set_z(-8.18);
	    }
	    obstacle->set_height(1.56);
	    obstacle->set_id("c88");
	}
}

void DetectCar(Pic &xu,interface::perception::PerceptionObstacles &perception_result,Vector trans)
{
	int i;
	for(i=0;i<xu.cc;i++)
	{
		if(xu.v[i].size())
			DetectSingleObject(xu.v[i],perception_result,trans);
	}
}

interface::perception::PerceptionObstacles Perception::RunPerception(
    const PointCloud& pc, const utils::Optional<cv::Mat>& image) {
  	

	interface::perception::PerceptionObstacles perception_result;
	//return perception_result;

	now=sxu+pic_cc;
	pic_cc++;
	if(pic_cc>=pic_num)
	{
		pic_cc=0;
		pic_cycle=1;
	}
	GetData(now,pc);
	DetectGround(*now,*(sxu+pic_num));
	now=sxu+pic_num;
	//GetSmooth(*now);
	GetUnion(*now);
	//PrintLump(*now,perception_result);
	DetectCar(*now,perception_result,now->trans);


  if (image) {
    // Remove me if you don't want to pause the program every time.
    cv::namedWindow("camera");
    imshow("camera", *image);
    cv::waitKey(0);
  }

  LOG(INFO) << "Perception done.";
  return perception_result;
}
