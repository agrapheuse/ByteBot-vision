from setuptools import find_packages, setup

package_name = 'vision_controller'

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
    maintainer='Elina Van der Taelen',
    maintainer_email='elina.vandertaelen@student.kdg.be',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_images_from_oakd = vision_controller.get_images_from_oakd:main'
        ],
    },
)
