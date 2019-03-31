package(default_visibility = ["//visibility:public"])

cc_library(
    name = "logger_agent",
    hdrs = ["logger_agent.h"],
    deps = [
        "//pnc/simulation:vehicle_agent",
        "//pnc/simulation:vehicle_agent_factory",
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "@glog",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "Chenyao2333_agent",
    hdrs = ["Chenyao2333_agent.h"],
    deps = [
        "//pnc/simulation:vehicle_agent",
        "//pnc/simulation:vehicle_agent_factory",
        "//pnc/agents/Chenyao2333:planning",
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "@glog",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "planning",
    hdrs = ["planning.h"],
    deps = [
        "//pnc/map:map_lib",
        "//common/utils/file:file",
        "//common/proto:cc_map_proto",
        "//common/proto:cc_map_lane_proto",
        "//common/proto:cc_geometry_proto",
        "//common/proto:cc_route_proto",
    ],
)