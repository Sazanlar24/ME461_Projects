from setuptools import find_packages, setup

package_name = 'project_pkg'

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
    maintainer='me461',
    maintainer_email='me461@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher  = project_pkg.publisher:main',
            'subscriber = project_pkg.subscriber:main',
            'gui = project_pkg.gui:main',
            'image_processing = project_pkg.image_processing:main',
            'marker = project_pkg.marker:main',  
        ],
    },
)
