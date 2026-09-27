from setuptools import setup
from glob import glob
setup(name='tram_odometry',version='0.1.0',packages=['tram_odometry'],
      data_files=[('share/ament_index/resource_index/packages',['resource/tram_odometry']),
                  ('share/tram_odometry',['package.xml']),
                  ('share/tram_odometry/launch',glob('launch/*.py')),
                  ('share/tram_odometry/config',glob('config/*'))],
      install_requires=['setuptools'],zip_safe=True,
      maintainer='Project author',maintainer_email='maintainer@example.invalid',
      description='Model-based reserve tram odometry',license='Proprietary',
      entry_points={'console_scripts':['odometry_node = tram_odometry.node:main']})
