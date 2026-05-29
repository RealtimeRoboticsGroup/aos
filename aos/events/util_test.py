from absl.testing import absltest

from aos.events import util
from aos.configuration_fbs_py.aos.Configuration import ConfigurationT, Configuration


class ConfigurationTest(absltest.TestCase):

    def setUp(self):
        self.config = util.ConfigurationBuffer(
            util.locate("aos/aos/events/event_loop_py_config.bfbs"))

    def test_configuration_fbs(self):
        configuration_fbs = self.config.c_to_fbs(Configuration,
                                                 self.config.c_root())
        assert self.config.fbs_to_c(configuration_fbs) == self.config.c_root()
        configuration_t = ConfigurationT.InitFromObj(configuration_fbs)
        for i, channel in enumerate(configuration_t.channels):
            assert channel.name == configuration_fbs.Channels(i).Name(), i


if __name__ == "__main__":
    absltest.main()
