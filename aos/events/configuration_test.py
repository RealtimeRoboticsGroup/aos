from absl.testing import absltest

from aos.events import util
from aos.events import configuration
from aos.configuration_fbs_py.aos.Configuration import ConfigurationT, Configuration
from aos.configuration_fbs_py.aos.Node import NodeT, Node


class ConfigurationTest(absltest.TestCase):

    def setUp(self):
        self.config = util.ConfigurationBuffer(
            util.locate(
                "aos/aos/testing/ping_pong/multinode_pingpong_config.bfbs"))

    def test_basics(self):
        configuration_t = self.config.root_fbs_t()
        self.assertEqual(len(configuration_t.nodes), 2)

        assert configuration.multi_node(self.config)
        for (node_fbs, node_t,
             name) in zip(configuration.get_nodes(self.config),
                          configuration_t.nodes, (b'devbox', b'remote')):
            node_fbs_t = NodeT.InitFromObj(node_fbs)
            re_node_fbs = configuration.get_node(self.config,
                                                 name.decode('utf-8'))
            assert self.config.fbs_to_c(re_node_fbs) == self.config.fbs_to_c(
                node_fbs)

            assert node_fbs_t.name == name, node_fbs_t.name
            assert node_fbs.Name() == name, node_fbs.Name()

        devbox = configuration.get_node(self.config, 'devbox')
        remote = configuration.get_node(self.config, 'remote')

        devbox_only = configuration.get_channel(
            self.config, '/devbox/aos', 'aos.message_bridge.ServerStatistics',
            'ping', devbox)
        assert configuration.channel_is_readable_on_node(
            self.config, devbox_only, devbox)
        assert not configuration.channel_is_readable_on_node(
            self.config, devbox_only, remote)
        assert configuration.channel_is_sendable_on_node(
            self.config, devbox_only, devbox)
        assert not configuration.channel_is_sendable_on_node(
            self.config, devbox_only, remote)

        remote_only = configuration.get_channel(
            self.config, '/remote/aos', 'aos.message_bridge.ServerStatistics',
            'ping', remote)
        assert not configuration.channel_is_readable_on_node(
            self.config, remote_only, devbox)
        assert configuration.channel_is_readable_on_node(
            self.config, remote_only, remote)
        assert not configuration.channel_is_sendable_on_node(
            self.config, remote_only, devbox)
        assert configuration.channel_is_sendable_on_node(
            self.config, remote_only, remote)

        devbox_to_remote = configuration.get_channel(
            self.config, '/devbox/aos', 'aos.message_bridge.Timestamp', 'ping',
            devbox)
        assert configuration.channel_is_readable_on_node(
            self.config, devbox_to_remote, devbox)
        assert configuration.channel_is_readable_on_node(
            self.config, devbox_to_remote, remote)
        assert configuration.channel_is_sendable_on_node(
            self.config, devbox_to_remote, devbox)
        assert not configuration.channel_is_sendable_on_node(
            self.config, devbox_to_remote, remote)

        remote_to_devbox = configuration.get_channel(
            self.config, '/remote/aos', 'aos.message_bridge.Timestamp', 'ping',
            remote)
        assert configuration.channel_is_readable_on_node(
            self.config, remote_to_devbox, devbox)
        assert configuration.channel_is_readable_on_node(
            self.config, remote_to_devbox, remote)
        assert not configuration.channel_is_sendable_on_node(
            self.config, remote_to_devbox, devbox)
        assert configuration.channel_is_sendable_on_node(
            self.config, remote_to_devbox, remote)


if __name__ == "__main__":
    absltest.main()
