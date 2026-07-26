from setuptools import find_packages, setup


package_name = 'preview_tools'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='jimmy',
    maintainer_email='jimmy@example.com',
    description='Lightweight mapping trajectory and coverage preview tools.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'coverage_analyzer = preview_tools.coverage_analyzer:main',
        ],
    },
)
