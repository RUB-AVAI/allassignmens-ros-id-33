from setuptools import setup

package_name = 'camera_package'

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
    maintainer='ubuntu',
    maintainer_email='avai@unconfigured.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_package.camera_node:main',
            'image_display_node = camera_package.image_display_node:main',
            'image_processing_node = camera_package.image_processing_node:main',
        ],
    },
)
