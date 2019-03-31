package(default_visibility = ["//visibility:public"])

cc_library(
    name = "agents",
    hdrs = ["agents.h"],
    srcs = ["agents.cc"],
    deps = [
        "//pnc/agents/faebdc:faebdc_agent",
        "//pnc/simulation:vehicle_agent_factory",
    ],
    alwayslink = 1,
)
