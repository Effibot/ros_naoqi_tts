from setuptools import setup

package_name = 'ros_naoqi_tts'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea efficace',
    maintainer_email='andrea.efficace1@gmail.com',
    description='TODO: Package description',
    license='LICENSE',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tts_node = ros_naoqi_tts.tts_node:main'
        ],
    },
)
