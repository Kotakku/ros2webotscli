from setuptools import find_packages
from setuptools import setup

package_name = 'ros2webotscli'

setup(
  name=package_name,
  version='0.1.0',
  packages=find_packages(exclude=['test']),
  data_files=[
    ('share/' + package_name, ['package.xml']),
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
  ],
  install_requires=['ros2cli'],
  zip_safe=True,
  maintainer='Takumi Odashima',
  maintainer_email='kotakkucu@gmail.com',
  url='',
  download_url='',
  keywords=[],
  classifiers=[
      'Environment :: Console',
      'Intended Audience :: Developers',
      'Programming Language :: Python',
  ],
  description='CLI for develop ROS2 + Webots',
  long_description="""""",
  license='',
  tests_require=['pytest'],
  entry_points={
        'ros2cli.command': [
            'webots = ros2webotscli.command.webots:WebotsCommand'
        ],
        'ros2webotscli.verb': [
            'pkg_create = ros2webotscli.verb.pkg_create:PkgCreateVerb',
            'run = ros2webotscli.verb.run:RunVerb'
        ],
    }
)