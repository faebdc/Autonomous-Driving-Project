// Copyright @2018 Pony AI Inc. All rights reserved.


	//×ªÍä¿ÉÄÜ»á×²µ½ 
	
	//ÔËÐÐÌ«ÂýÁË 


#pragma once

#include "pnc/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "pnc/simulation/vehicle_agent_factory.h"



#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "pnc/simulation/vehicle_agent.h"
#include "pnc/simulation/vehicle_agent_factory.h"




#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <vector>

#include "common/proto/geometry.pb.h"
#include "common/proto/map.pb.h"
#include "common/proto/map_lane.pb.h"
//#include "common/proto/route.pb.h"
#include "common/utils/file/file.h"

#include "pnc/map/map_lib.h"





namespace sample {



const double INF = 1e8;
const double eps = 1e-7;

struct Node {
  double x, y;

  Node() {}
  template <typename T>
  Node(const T& p) {
    x = p.x();
    y = p.y();
  }
};

double dist(const Node& a, const Node& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return dx * dx + dy * dy;
}




class Graph {
 public:
  Graph() {}
  int add_node(const Node& n) {
    Nodes.push_back(n);
    G.push_back(std::vector<int>());
    return Nodes.size() - 1;
  }
  void add_edge(int a, int b) { G[a].push_back(b); }

  std::vector<Node> shortest_path(Node start, Node end) {
    int sid = -1, eid = -1;
    for (int i = 0; i < Nodes.size(); i++) {
      if (sid == -1 || dist(Nodes[i], start) < dist(Nodes[sid], start)) {
        sid = i;
      }
      if (eid == -1 || dist(Nodes[i], end) < dist(Nodes[eid], end)) {
        eid = i;
      }
    }

    // std::cout << sid << " " << eid << std::endl;

    std::vector<int> fa(Nodes.size(), -1);
    std::vector<double> d(Nodes.size(), INF);
    std::priority_queue<std::pair<double, int> > q;
    d[sid] = 0;
    fa[sid] = sid;
    q.push(std::make_pair(0.0, sid));

    while (q.size()) {
      double c = -q.top().first;
      int u = q.top().second;
      // std::cout << u << std::endl;
      q.pop();

      if (c > d[u] + eps) continue;

      for (int v : G[u]) {
        double nc = c + dist(Nodes[u], Nodes[v]);
        if (nc + eps < d[v]) {
          d[v] = nc;
          fa[v] = u;
          q.push(std::make_pair(-nc, v));
        }
      }
    }

    CHECK(fa[eid] != -1);
    std::vector<Node> path;
    for (int u = eid; u != sid; u = fa[u]) {
      path.push_back(Nodes[u]);
    }
    path.push_back(Nodes[sid]);

    std::reverse(path.begin(), path.end());
    return path;
  }

 private:
  std::vector<Node> Nodes;
  std::vector<std::vector<int> > G;
};

interface::geometry::Point3D get_start(const interface::map::Lane& lane) {
  CHECK(lane.central_line().point_size() >= 1);
  return lane.central_line().point(0);
}

interface::geometry::Point3D get_end(const interface::map::Lane& lane) {
  CHECK(lane.central_line().point_size() >= 1);
  return lane.central_line().point(lane.central_line().point_size() - 1);
}

bool equal_Point3D(const interface::geometry::Point3D& a, const interface::geometry::Point3D& b) {
  // cout << std::abs(a.x() - b.x()) << " " <<  std::abs(a.y() - b.y()) << " " << std::abs(a.z() -
  // b.z()) << endl;
  return std::abs(a.x() - b.x()) < eps && std::abs(a.y() - b.y()) < eps &&
         std::abs(a.z() - b.z()) < eps;
}

void process_map(interface::map::Map& map_proto) {
  for (int i = 0; i < map_proto.lane_size(); i++) {
    auto* l = map_proto.mutable_lane(i);

    auto start = get_start(*l);
    auto end = get_end(*l);

    // find predecessor
    for (int j = 0; j < map_proto.lane_size(); j++) {
      if (i == j) continue;
      auto n_start = get_start(map_proto.lane(j));
      auto n_end = get_end(map_proto.lane(j));

      // successor
      if (equal_Point3D(end, n_start)) {
        auto* p = l->add_successor();
        p->CopyFrom(map_proto.lane(j).id());
        // cout << "asdfas"<<endl;
      }

      // predeceesor
      if (equal_Point3D(start, n_end)) {
        auto* p = l->add_predecessor();
        p->CopyFrom(map_proto.lane(j).id());
      }
    }
  }
}

Graph load_graph(interface::map::Map& map_proto) {
  Graph g;
  pnc::map::MapLib maplib;
  map_proto = maplib.map_proto();
  process_map(map_proto);

  std::map<std::string, int> lane_begin, lane_end;

  for (const auto& lane : map_proto.lane()) {
    CHECK(lane.central_line().point_size() >= 1);

    int last_id = -1;
    for (const auto& p : lane.central_line().point()) {
      int cur_id = g.add_node(Node(p));
      if (last_id >= 0)
        g.add_edge(last_id, cur_id);
      else {
        lane_begin[lane.id().id()] = cur_id;
      }
      last_id = cur_id;
    }

    lane_end[lane.id().id()] = last_id;
  }

  for (const auto& lane : map_proto.lane()) {
    for (const auto& id : lane.successor()) {
      // std::cout << lane.id().id() << " " << id.id() << std::endl;
      g.add_edge(lane_end[lane.id().id()], lane_begin[id.id()]);
    }
  }

  return g;
}


















// A sample vehicle agent for route 1
// This agent will always run in straight. It will accelerate to the speed slightly over 5m/s,
// and keeps running with the speed over 5m/s until reaches destination.

const double PI = 3.1415926535;



class SampleVehicleAgent : public simulation::VehicleAgent {
 public:
  explicit SampleVehicleAgent(const std::string& name) : VehicleAgent(name) {}

  void Initialize(const interface::agent::AgentStatus&  agent_status ) /*override*/ 
  {
    // Nothing to initialize
    g = load_graph(map_proto);
    path = g.shortest_path(vehicle_position(agent_status),
                           agent_status.route_status().destination());
    path_index = 0;

    allow_yellow=-1;
  }





  interface::control::ControlCommand RunOneIteration(
      const interface::agent::AgentStatus& agent_status) override {
      	
      	
      	double STOP_DIS = 10;
      	double TRAFFIC_DIS = 13;
      	double STOP_EPS = 0;
      	double TRAFFIC_STOP_EPS = 7;
      	double MAX_V = 5 * 2;
      	double RATE = 0.226667 * 1.5;
      	double PATH_NEXT_POINT_DIS = 3 * 1.5;
      	double CAR_DANGER_DIS = 20;
      	double PERSON_DANGER_DIS = 20;
      	double CAR_LINE_DIS = 5;
      	double PERSON_LINE_DIS = 4;
      	double DANGER_EPS = 3;
      	double DANGER_LINE_DIS = 2;
      	double HALFCYCLE = 23;
      	
      	
      	interface::geometry::Point3D zero;
      	zero.set_x(0.0);
      	zero.set_y(0.0);
      	zero.set_z(0.0);
      	
      	

      	
/*******New Dest*********/      	

	    // find next point on the path
	    while (path_index < path.size()) {
	      if (CalcDistance(vehicle_position(agent_status), to_point3d(path[path_index])) <
	          1.000 * PATH_NEXT_POINT_DIS) {
	        path_index++;
	      } else {
	        break;
	      }
	    }

	    interface::geometry::Point3D dest = agent_status.route_status().destination();

	    if (path_index < path.size()) {
	      dest = to_point3d(path[path_index]);
	    } else {
	      // std::cout << "???" << std::endl;
	    }


      	
      	if(agent_status.route_status().is_new_request())
      	{
      		path.clear();
		    path = g.shortest_path(vehicle_position(agent_status),
		                           agent_status.route_status().destination());
		    path_index = 0;
      	}
      	


      	
      	
      	interface::control::ControlCommand command;
      	  	
    double velocity = CalcVelocity(agent_status.vehicle_status().velocity());
    interface::geometry::Point3D nowp= vehicle_position(agent_status);
    interface::geometry::Vector3d aim = Jian(dest,nowp);



/******* reach dest ***********/

    
    // Vehicle's current position reaches the destination
    if (CalcDistance(nowp,  agent_status.route_status().destination()) <
        (velocity/(0.600*MAX_V)) * STOP_DIS + STOP_EPS ) {  // 15 is picked according to table.txt
      position_reached_destination_ = true;
    }
    else
    {
    	position_reached_destination_ = false;
    }



 /******* reach red light ***********/
    

    position_reached_red_light_=false;
    for(const auto &trc : agent_status.perception_status().traffic_light())
    for(const auto &tra : trc.single_traffic_light_status())
    {
    	if(tra.color()==interface::map::Bulb::RED || 
    		  (tra.color()==interface::map::Bulb::YELLOW && agent_status.simulation_status().simulation_time()>allow_yellow) )
    	for(const auto &trd : map_proto.traffic_light())
    	if(tra.id().id()==trd.id().id())
    	{
    		interface::geometry::Point3D last;
    		last.set_x(-1e9);
    		last.set_y(-1e9);
    		last.set_z(-1e9);
    		for(const auto &trb : trd.stop_line().point())
    		{
				if(position_reached_red_light_)
					break;
    			if(last.x()>-1e-8)
    			{
				    if (CalcDistance(nowp, trb) <
				        (velocity/(0.600*MAX_V)) *  TRAFFIC_DIS + TRAFFIC_STOP_EPS) {  // 15 is picked according to table.txt
				        
				        /*for(int i=path_index;i<path.size();i++)
				        {
				        	if(CalcDistance(vehicle_position(agent_status),
				                     to_point3d(path[i]) ) < TRAFFIC_DIS ) 
				            {
				            	if(CalcDistance( MidPoint( trb , last ) ,
				                     to_point3d(path[i]) ) < 3 )
				            	{
					            	position_reached_red_light_ = true;
					            	break;
					            }
				            }
				            break;
				        }*/
				        if( LineDis( aim, Jian( MidPoint( trb , last ),nowp ) ) < 3 )
				        {
				        	if((std::abs(aim.x())>=20*std::abs(aim.y()) || std::abs(aim.y())>=20*std::abs(aim.x())) 
				        		 && LineUp(aim,Jian(MidPoint( trb , last),nowp)) )
				        	{
				        		position_reached_red_light_ = true;
				        		if(tra.color()==interface::map::Bulb::RED)
				        			allow_yellow = agent_status.simulation_status().simulation_time()+ 1.000*HALFCYCLE;
				        	}
				        }
				      
				    }
				}
				last=trb;
			}
			break;
		}
	}
	
	
	
/******* reach object ***********/

	
	
	position_reached_object_ = false;
	almost_reached_object_ = false;
	double minratio=1.0;
	
	for(const auto &ob : agent_status.perception_status().obstacle())
	{
		if(ob.type()==interface::perception::CAR)
		{
			for(const auto &p : ob.polygon_point())
			{
				if (CalcDistance(nowp, p) <
         			 1.000 * CAR_DANGER_DIS)
         		if ( LineDis( aim ,
				 Jian(p,nowp ) ) < CAR_LINE_DIS  )
				if( LineRight( aim,
				 Jian(p,nowp ) )  )
				if( LineUp( aim,
				 Jian(p,nowp ) )  )
         		{
         			position_reached_object_=true;
         			if (CalcDistance(nowp, p) <
         			 1.000 * DANGER_EPS)
         			 	almost_reached_object_ = true;
         			 minratio=std::min(minratio, (CalcDistance(nowp, p)-1.000 * DANGER_EPS) / (1.200 * PERSON_DANGER_DIS));
         		}
			}
		}
		if(ob.type()==interface::perception::PEDESTRIAN || ob.type()==interface::perception::CYCLIST)
		{
			for(const auto &p : ob.polygon_point())
			{
				if (CalcDistance(nowp, p) <
         			 1.000 * PERSON_DANGER_DIS)
         		if ( LineDis( aim,
				 Jian(p,nowp )) < PERSON_LINE_DIS  )
				if( LineUp( aim,
				 Jian(p,nowp ) )  )
         		{
         			position_reached_object_=true;
         			if (CalcDistance(nowp, p) < 1.000 * DANGER_EPS 
         				&& LineDis( aim, Jian(p,nowp )) < DANGER_LINE_DIS)
         			 	almost_reached_object_ = true;
         			 minratio=std::min(minratio, (CalcDistance(nowp, p)-1.000 * DANGER_EPS) / (1.200 * PERSON_DANGER_DIS));
         		}
         		break;
			}
		}
	}
	MAX_V *=minratio;
	
	

/******* too large velocity ***********/

    
    // Vehicle's current velocity reaches 5 m/s
    if (velocity > 1.000 * MAX_V) {
      velocity_reached_threshold_ = true;
    }
    else
    {
    	velocity_reached_threshold_ = false;
    }
    
    if (velocity > 1.100 * MAX_V)
    	velocity_too_large_ = true;
    else
    	velocity_too_large_ = false;
    
    

    if (position_reached_destination_) {
      command.set_brake_ratio(1.000 * RATE);  // 0.226667 is picked according to table.txt
    } else if(position_reached_red_light_)
      command.set_brake_ratio(1.000 * RATE);  // 0.226667 is picked according to table.txt
	  else if(velocity_too_large_)
      command.set_brake_ratio(std::min(1.000, (velocity/MAX_V-0.1) * RATE));  // 0.226667 is picked according to table.txt
	  else if(almost_reached_object_)
      command.set_brake_ratio(2.000 * RATE);  // 0.226667 is picked according to table.txt
	  else if(position_reached_object_)
	  	;
	  else {
      if (!velocity_reached_threshold_) {
        command.set_throttle_ratio(1.000 * RATE);  // 0.226667 is picked according to table.txt
      }
    }



    double angle = CalcAngle(agent_status.vehicle_status(), dest);

    // the angle of steering is calculated by angle between the velocity and destination
    if (std::abs(angle) > 0.1 && std::abs(angle) < 1) {
      angle *= 3;
    }
    command.set_steering_angle(angle * 5);


    // log the velocitys
    /*if (std::abs(velocity) > 0.05) {
      std::ofstream output;
      output.open("/tmp/velocity.txt", std::ofstream::out | std::ofstream::app);
      output << agent_status.simulation_status().simulation_time() << " " << velocity << std::endl;
      output.close();
    }*/

    return command;
      	
      	
      	
      	
      	
      	
      	
      	
      	
      	
      	
      	
      	
      	/****
      	
      	
    //interface::control::ControlCommand command;
    // Vehicle's current position reaches the destination
    if (CalcDistance(nowp,
                     agent_status.route_status().destination()) < 3.0) {
      position_reached_destination_ = true;
    }
    // Vehicle's current velocity reaches 5 m/s
    if (CalcVelocity(agent_status.vehicle_status().velocity()) > 5) {
      velocity_reached_threshold_ = true;
    }

    if (position_reached_destination_) {
      // Set maximum brake ratio to stop the vehicle
      command.set_brake_ratio(1.0);
    } else {
      if (!velocity_reached_threshold_) {
        // Set throttle ratio to accelerate
        command.set_throttle_ratio(0.3);
      }
    }

    PublishVariable("key1", "var1");
    PublishVariable("key2", "var2", utils::display::Color::Red());
    PublishVariable("key3", "var3", utils::display::Color::Red(), utils::display::Color::Green());

    return command;
    
    **********/
    
  }

 private:
  interface::geometry::Point3D to_point3d(Node n) {
    interface::geometry::Point3D p;
    p.set_x(n.x);
    p.set_y(n.y);
    p.set_z(0);
    return p;
  }
  interface::geometry::Point3D vtp(const interface::geometry::Vector3d& n) {
    interface::geometry::Point3D p;
    p.set_x(n.x());
    p.set_y(n.y());
    p.set_z(n.z());
    return p;
  }
  interface::geometry::Vector3d ptv(const interface::geometry::Point3D& n) {
    interface::geometry::Vector3d p;
    p.set_x(n.x());
    p.set_y(n.y());
    p.set_z(n.z());
    return p;
  }

  double CalcDistance(const interface::geometry::Point3D& position,
                      const interface::geometry::Point3D& destination) {
    double sqr_sum =
        math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
    ;
    return std::sqrt(sqr_sum);
  }
  
  interface::geometry::Point3D MidPoint (const interface::geometry::Point3D& a,
                      const interface::geometry::Point3D& b){
    interface::geometry::Point3D c;
    c.set_x(0.5*(a.x()+b.x()));
    c.set_y(0.5*(a.y()+b.y()));
    c.set_z(0.5*(a.z()+b.z()));
    return c;
  }
  
  interface::geometry::Vector3d Jian (const interface::geometry::Point3D& a,
                      const interface::geometry::Point3D& b){
    interface::geometry::Vector3d c;
    c.set_x((a.x()-b.x()));
    c.set_y((a.y()-b.y()));
    c.set_z((a.z()-b.z()));
    return c;
  }
  
  double ChaJi2d (const interface::geometry::Vector3d& a,
                      const interface::geometry::Vector3d& b){
    double res=0.0;
    res+=a.x()*b.y();
    res-=a.y()*b.x();
    return res;
  }
  
  double DianJi2d (const interface::geometry::Vector3d& a,
                      const interface::geometry::Vector3d& b){
    double res=0.0;
    res+=a.x()*b.x();
    res+=a.y()*b.y();
    return res;
  }
  
  double Len2d (const interface::geometry::Vector3d& a ){
    double res=0.0;
    res+=a.x()*a.x();
    res+=a.y()*a.y();
    res=std::sqrt(res);
    return res;
  }
  
  double LineDis (const interface::geometry::Vector3d& a,
                      const interface::geometry::Vector3d& b){
    double res;
    res=ChaJi2d(a,b);
    res/=Len2d(a);
    if(res<0)
    	res=-res;
    return res;
  }
  
  bool LineRight(const interface::geometry::Vector3d& a,
                      const interface::geometry::Vector3d& b){
    double res;
    res=ChaJi2d(a,b);
	return (res<0);
  }
  
  bool LineUp(const interface::geometry::Vector3d& a,
                      const interface::geometry::Vector3d& b){
    double res;
    res=DianJi2d(a,b);
	return (res>0);
  }

  interface::geometry::Point3D vehicle_position(const interface::agent::AgentStatus& agent_status)
  {
  	double width = 4.5;
  	interface::geometry::Vector3d res = agent_status.vehicle_status().position();
  	interface::geometry::Vector3d resx = agent_status.vehicle_status().velocity();
  	double val = Len2d(res);
  	res.set_x( res.x() + width*resx.x()/val );
  	res.set_y( res.y() + width*resx.y()/val );
  	res.set_z( res.z() + width*resx.z()/val );
  	return vtp(res);
  }
  

  double CalcVelocity(const interface::geometry::Vector3d& velocity) {
    double sqr_sum = math::Sqr(velocity.x()) + math::Sqr(velocity.y());
    ;
    return std::sqrt(sqr_sum);
  }

  double CalcAngle(const interface::agent::VehicleStatus& vehicle_status,
                   const interface::geometry::Point3D& destination) {
    Eigen::Vector2d o, dest;
    o[0] = vehicle_status.velocity().x();
    o[1] = vehicle_status.velocity().y();

    dest[0] = destination.x() - vehicle_status.position().x();
    dest[1] = destination.y() - vehicle_status.position().y();

    double ret = std::atan2(dest.y(), dest.x()) - std::atan2(o.y(), o.x());
    if (std::abs(ret) > PI) {
      if (ret > 0)
        ret = ret - 2 * PI;
      else
        ret = ret + 2 * PI;
    }
    return ret;
  }

  // Whether vehicle's current position reaches the destination
  bool position_reached_destination_ = false;
  // Whether vehicle's current velocity reaches 5 m/s
  bool velocity_reached_threshold_ = false;
  
  bool velocity_too_large_ = false;
  
  double allow_yellow;
  bool position_reached_red_light_ = false;
  
  bool position_reached_object_ = false;
  bool almost_reached_object_ = false;

  Graph g;
  std::vector<Node> path;
  int path_index;
  interface::map::Map map_proto;
};





}  // namespace sample
