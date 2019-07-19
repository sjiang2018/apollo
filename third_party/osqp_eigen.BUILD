package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "osqp_eigen",
    srcs = glob(["lib/*.so"]),
    hdrs = glob(["*"]),
    include_prefix = "OsqpEigen",
    includes = [
        ".",
        "include",
        "include/OsqpEigen",
    ],
    linkopts = [
        "-Llib",
    ],
    linkstatic = False,
    deps = [
        "@osqp",
        "@eigen"
    ],
)