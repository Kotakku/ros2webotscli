import os
from ros2cli.verb import VerbExtension
from webots_ros2_driver.utils import get_webots_home
import subprocess

class RunVerb(VerbExtension):
    """Run webots."""

    def main(self, *, args):
      subprocess.run([os.path.join(get_webots_home(), "webots")])