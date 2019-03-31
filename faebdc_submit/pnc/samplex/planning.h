#pragma once

#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <vector>

#include "common/proto/geometry.pb.h"
#include "common/proto/map.pb.h"
#include "common/proto/map_lane.pb.h"
#include "common/proto/route.pb.h"
#include "common/utils/file/file.h"

#include "pnc/map/map_lib.h"

namespace Chenyao2333 {

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

Graph load_graph() {
  Graph g;
  pnc::map::MapLib maplib;
  interface::map::Map map_proto = maplib.map_proto();
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

}  // namespace Chenyao2333
