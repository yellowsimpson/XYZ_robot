import os
from glob import glob
from setuptools import setup, find_packages  # find_packages 추가!

package_name = 'urdf_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # URDF 및 xacro 파일 포함
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 다른 파일들도 필요한 경우 추가
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shim',
    maintainer_email='shimsungwhan@naver.com',
    description='URDF tutorial package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
