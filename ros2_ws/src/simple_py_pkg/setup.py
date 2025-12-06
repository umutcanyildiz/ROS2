from setuptools import find_packages, setup

package_name = 'simple_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='umut',
    maintainer_email='umutyildiz2647@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "py_node = simple_py_pkg.simple_py_node:main",
            "counter_node = simple_py_pkg.counter_node:main", #muhakkak buraya da eklenir
            "channel_node = simple_py_pkg.televison:main",
            "remote_control_node = simple_py_pkg.remote_controller:main",
            "add_two_ints_server_node = simple_py_pkg.add_two_ints_server:main",
            "add_two_ints_client_node = simple_py_pkg.add_two_ints_client:main",
            "add_two_ints_client_oop_node = simple_py_pkg.add_two_ints_client_oop:main",

        ],
    },
)
