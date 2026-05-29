"""Wrappers for aos/configuration.h.

The functions in this module rely on `ConfigurationBuffer` to track Python
ownership of the underlying buffer. This means that all functions take this as
an argument, even the ones which don't in C++."""

from aos.events.event_loop_c import ffi, lib
from aos.configuration_fbs_py.aos.Configuration import Configuration, Node, Channel
from aos.events.util import ConfigurationBuffer


def multi_node(config: ConfigurationBuffer) -> bool:
    return lib.aos_configuration_multi_node(
        ffi.cast('aos_configuration_t *', config.c_root()))


def get_nodes(config: ConfigurationBuffer) -> list[Node]:
    c_result = ffi.new('const aos_node_t *[]', 100)
    size = lib.aos_configuration_get_nodes(
        ffi.cast('aos_configuration_t *', config.c_root()), c_result,
        len(c_result))
    assert size <= len(
        c_result
    ), 'Too many nodes, please increase the maximum number of nodes in the Python API'
    result = list()
    for i in range(size):
        result.append(config.c_to_fbs(Node, c_result[i]))
    return result


def channel_is_sendable_on_node(config: ConfigurationBuffer, channel: Channel,
                                node: Node) -> bool:
    c_node = ffi.NULL if node is None else ffi.cast('aos_node_t *',
                                                    config.fbs_to_c(node))
    return lib.aos_configuration_channel_is_sendable_on_node(
        ffi.cast('aos_channel_t *', config.fbs_to_c(channel)), c_node)


def channel_is_readable_on_node(config: ConfigurationBuffer, channel: Channel,
                                node: Node) -> bool:
    c_node = ffi.NULL if node is None else ffi.cast('aos_node_t *',
                                                    config.fbs_to_c(node))
    return lib.aos_configuration_channel_is_readable_on_node(
        ffi.cast('aos_channel_t *', config.fbs_to_c(channel)), c_node)


def get_channel(config: ConfigurationBuffer, name: str, type_name: str,
                application_name: str, node: Node) -> Channel:
    c_name = ffi.from_buffer(name.encode('utf-8'))
    c_type = ffi.from_buffer(type_name.encode('utf-8'))
    c_application_name = ffi.from_buffer(application_name.encode('utf-8'))
    c_node = ffi.NULL if node is None else ffi.cast('aos_node_t *',
                                                    config.fbs_to_c(node))
    return config.c_to_fbs(
        Channel,
        lib.aos_configuration_get_channel(
            ffi.cast('aos_configuration_t *', config.c_root()), c_name,
            len(c_name), c_type, len(c_type), c_application_name,
            len(c_application_name), c_node))


def get_node(config: ConfigurationBuffer, name: str) -> Node:
    c_name = ffi.from_buffer(name.encode('utf-8'))
    return config.c_to_fbs(
        Node,
        lib.aos_configuration_get_node(
            ffi.cast('aos_configuration_t *', config.c_root()), c_name,
            len(c_name)))
