from io import StringIO
import os
import sys

import em
try:
    import importlib.resources as importlib_resources
except ModuleNotFoundError:
    import importlib_resources

import ros2pkg
# from ros2pkg.api.create import create_package_environment
# from ros2pkg.api.create import populate_ament_cmake
# from ros2pkg.api.create import populate_ament_python
# from ros2pkg.api.create import populate_cmake
# from ros2pkg.api.create import populate_cpp_library
# from ros2pkg.api.create import populate_cpp_node
# from ros2pkg.api.create import populate_python_libary
# from ros2pkg.api.create import populate_python_node


from ros2pkg.api.create import _expand_template

def _create_template_file(
    template_subdir, template_file_name, output_directory, output_file_name, template_config
):
    full_package = 'ros2webotscli.resource.' + template_subdir
    with importlib_resources.path(full_package, template_file_name) as path:
        template_path = str(path)
    if not os.path.exists(template_path):
        raise FileNotFoundError('template not found:', template_path)

    output_file_path = os.path.join(output_directory, output_file_name)

    print('creating', output_file_path)
    _expand_template(template_path, template_config, output_file_path)
    

def create_package_environment(package, destination_directory):
    return ros2pkg.api.create.create_package_environment(package, destination_directory)

def populate_ament_cmake(package, package_directory):
    cmakelists_config = {
        'package_name': package.name,
        'dependencies': [str(dep) for dep in package.build_depends],
    }
    _create_template_file(
        'ament_cmake',
        'CMakeLists.txt.em',
        package_directory,
        'CMakeLists.txt',
        cmakelists_config)


def populate_plugin_description_xml(package, package_directory, cpp_class_name):
    plugin_desc_config = {
        'package_name': package.name,
        'class_name': cpp_class_name
    }
    _create_template_file(
        'xml',
        'plugin.xml.em',
        package_directory,
        package.name + '.xml',
        plugin_desc_config)

def populate_cpp_library(package, source_directory, include_directory, cpp_class_name):
    cpp_header_config = {
        'package_name': package.name,
        'class_name': cpp_class_name,
    }
    _create_template_file(
        'cpp',
        'header.hpp.em',
        include_directory,
        package.name + '.hpp',
        cpp_header_config)

    cpp_library_config = {
        'package_name': package.name,
        'class_name': cpp_class_name
    }
    _create_template_file(
        'cpp',
        'library.cpp.em',
        source_directory,
        package.name + '.cpp',
        cpp_library_config)

    visibility_config = {
        'package_name': package.name.upper(),
    }
    _create_template_file(
        'cpp',
        'visibility_control.h.em',
        include_directory,
        'visibility_control.h',
        visibility_config)
