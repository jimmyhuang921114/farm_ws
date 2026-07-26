from setuptools import setup
p='sensor_health_monitor'
setup(name=p,version='0.1.0',packages=[p],data_files=[('share/ament_index/resource_index/packages',['resource/'+p]),('share/'+p,['package.xml'])],install_requires=['setuptools'],zip_safe=True,maintainer='jimmy',maintainer_email='jimmy@localhost',description='Low-overhead sensor topic health monitor',license='Apache-2.0',entry_points={'console_scripts':['sensor_health_monitor = sensor_health_monitor.monitor:main']})
