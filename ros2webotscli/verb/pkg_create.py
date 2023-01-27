import getpass
import os
import shutil
import subprocess
import sys

from catkin_pkg.package import Dependency
from catkin_pkg.package import Export
from catkin_pkg.package import Package
from catkin_pkg.package import Person

from ros2cli.verb import VerbExtension

from ros2webotscli.api.create import (create_package_environment,
                                        populate_ament_cmake,
                                        populate_plugin_description_xml,
                                        populate_cpp_library,
                                        create_resource_folder,
                                        populate_launch,
                                        populate_urdf,
                                        populate_world)

class PkgCreateVerb(VerbExtension):
    """Create ROS2 + Webots plugin template package."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'package_name',
            help='The package name')
        parser.add_argument(
            'class_name',
            help='name of the empty class')
        parser.add_argument(
            '--package-format',
            '--package_format',
            type=int,
            default=3,
            choices=[2, 3],
            help='The package.xml format.')
        parser.add_argument(
            '--description',
            default='TODO: Package description',
            help='The description given in the package.xml')
        parser.add_argument(
            '--license',
            default='TODO: License declaration',
            help='The license attached to this package; this can be an arbitrary string, but a '
                 'LICENSE file will only be generated if it is one of the supported licenses '
                 "(pass '?' to get a list)")
        parser.add_argument(
            '--destination-directory',
            default=os.curdir,
            help='Directory where to create the package directory')
        parser.add_argument(
            '--build-type',
            default='ament_cmake',
            choices=['ament_cmake', 'ament_python'],
            help='The build type to process the package with')
        parser.add_argument(
            '--dependencies',
            nargs='+',
            default=[],
            help='list of dependencies')
        parser.add_argument(
            '--maintainer-email',
            help='email address of the maintainer of this package'),
        parser.add_argument(
            '--maintainer-name', default=getpass.getuser(),
            help='name of the maintainer of this package'),
        # parser.add_argument(
        #     '--node-name',
        #     help='name of the empty executable')
        parser.add_argument(
            '--library-name',
            help='name of the empty library')


    def main(self, *, args):
        maintainer = Person(args.maintainer_name)

        if args.maintainer_email:
            maintainer.email = args.maintainer_email
        else:
            # try getting the email from the global git config
            git = shutil.which('git')
            if git is not None:
                p = subprocess.Popen(
                    [git, 'config', '--global', 'user.email'],
                    stdout=subprocess.PIPE)
                resp = p.communicate()
                email = resp[0].decode().rstrip()
                if email:
                    maintainer.email = email
            if not maintainer.email:
                maintainer.email = maintainer.name + '@todo.todo'

        buildtool_depends = []
        if args.build_type == 'ament_cmake':
            if args.library_name:
                buildtool_depends = ['ament_cmake_ros']
            else:
                buildtool_depends = ['ament_cmake']

        test_dependencies = []
        if args.build_type == 'ament_cmake':
            test_dependencies = ['ament_lint_auto', 'ament_lint_common']
        if args.build_type == 'ament_python':
            test_dependencies = ['ament_copyright', 'ament_flake8', 'ament_pep257',
                                 'python3-pytest']

        if args.build_type == 'ament_python' and args.package_name == 'test':
            # If the package name is 'test', there will be a conflict between
            # the directory the source code for the package goes in and the
            # directory the tests for the package go in.
            return "Aborted since 'ament_python' packages can't be named 'test'. Please " + \
                'choose a different package name.'

        if args.build_type == 'ament_cmake':
            if not 'rclcpp' in args.dependencies:
                args.dependencies.append('rclcpp')
            if not 'webots_ros2_driver' in args.dependencies:
                args.dependencies.append('webots_ros2_driver')
            if not 'pluginlib' in args.dependencies:
                args.dependencies.append('pluginlib')
            

        package = Package(
            package_format=args.package_format,
            name=args.package_name,
            version='0.0.0',
            description=args.description,
            maintainers=[maintainer],
            licenses=[args.license],
            buildtool_depends=[Dependency(dep) for dep in buildtool_depends],
            build_depends=[Dependency(dep) for dep in args.dependencies],
            test_depends=[Dependency(dep) for dep in test_dependencies],
            exports=[Export('build_type', content=args.build_type)]
        )

        package_path = os.path.join(args.destination_directory, package.name)
        if os.path.exists(package_path):
            return '\nAborted!\nThe directory already exists: ' + package_path + '\nEither ' + \
                'remove the directory or choose a different destination directory or package name'

        print('going to create a new package')
        print('package name:', package.name)
        print('destination directory:', os.path.abspath(args.destination_directory))
        print('package format:', package.package_format)
        print('version:', package.version)
        print('description:', package.description)
        print('maintainer:', [str(maintainer) for maintainer in package.maintainers])
        print('licenses:', package.licenses)
        print('build type:', package.get_build_type())
        print('dependencies:', [str(dependency) for dependency in package.build_depends])
        print('library_name:', args.library_name)
        print('class_name:', args.class_name)
        

        package_directory, source_directory, include_directory = \
            create_package_environment(package, args.destination_directory)
        if not package_directory:
            return 'unable to create folder: ' + args.destination_directory
            

        if args.build_type == 'ament_cmake':
            # generate CMakeLists.txt
            populate_ament_cmake(package, package_directory)

            # generate plugin_description.xml
            populate_plugin_description_xml(package, package_directory, args.class_name)

            # generate .hpp, .cpp file
            if not source_directory or not include_directory:
                return 'unable to create source or include folder in ' + \
                        args.destination_directory
            populate_cpp_library(
                package,
                source_directory,
                include_directory,
                args.class_name
            )

            # create folders
            launch_directory, worlds_directory, resource_directory = create_resource_folder(package, args.destination_directory)

            # generate launch.py
            populate_launch(package, launch_directory) 

            # generate robot.urdf
            populate_urdf(package, args.class_name, resource_directory)

            populate_world(worlds_directory)

        # if args.build_type == 'ament_python':
        #     if not source_directory:
        #         return 'unable to create source folder in ' + args.destination_directory
        #     populate_ament_python(package, package_directory, source_directory, node_name)
        #     if node_name:
        #         populate_python_node(package, source_directory, node_name)
        #     if library_name:
        #         populate_python_libary(package, source_directory, library_name)