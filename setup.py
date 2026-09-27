from glob import glob

from setuptools import find_packages, setup

package_name = 'tram_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/json', glob('json/*.json')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='viktor',
    maintainer_email='viktor@todo.todo',
    description='Резервная одометрия трамвая по модели динамики (без GNSS/IMU)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'train_model_ros = tram_model.train_model_ros:main',
            'simple_model_ros = tram_model.simple_model_ros:main',
            'velocity_delta = tram_model.velocity_delta:main',
        ],
    },
)
