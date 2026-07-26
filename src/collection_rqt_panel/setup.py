from setuptools import setup


package_name = 'collection_rqt_panel'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/ament_index/resource_index/rqt_gui__pluginlib__plugin',
            ['resource/rqt_gui__pluginlib__plugin/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
        ('share/' + package_name + '/resource', ['resource/collection_panel.ui']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='jimmy',
    maintainer_email='jimmy@example.com',
    description='RQT collection, sensor health, and mapping coverage panel.',
    license='Apache-2.0',
)
