package(default_visibility = ["//visibility:public"])

# Whether a platform can run //aos/events:aio's io_uring backend (kernel >=
# 6.1, 6.12 recommended).  Both backends always build on Linux; this picks which one
# --aio_backend defaults to (see AOS_AIO_DEFAULT_BACKEND in //aos/events:aio).
#
# Defaults to :lacks_support so an out-of-tree platform that never says gets
# the epoll backend, which runs anywhere.
constraint_setting(
    name = "io_uring_support",
    default_constraint_value = ":lacks_support",
)

constraint_value(
    name = "has_support",
    constraint_setting = ":io_uring_support",
)

constraint_value(
    name = "lacks_support",
    constraint_setting = ":io_uring_support",
)
