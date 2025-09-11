from setuptools import setup

package_name = 'my_topic_demo_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Python topic pub/sub demo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_topic_demo_py.talker:main',
            'listener = my_topic_demo_py.listener:main',
        ],
    },
)
