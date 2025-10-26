from setuptools import find_packages, setup

package_name = 't1_keyboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/camera_viewer_launch.py',
            'launch/k1_camera_streamer_launch.py',
            'launch/camera_stream_demo_launch.py',
            'launch/usb_camera_stream_launch.py',
            'launch/k1_camera_agent_launch.py',
            'launch/gemini_robot_controller_launch.py',
            'launch/gemini_with_bridge_launch.py',
            'launch/voice_controlled_robot_launch.py',
            'launch/simple_robot_launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/camera_viewer.rviz',
            'config/k1_camera_streamer.yaml',
            'config/gemini_robot.rviz'
        ]),
    ],
    install_requires=[
        'setuptools', 
        'websockets', 
        'flask', 
        'opencv-python', 
        'requests', 
        'netifaces', 
        'PyYAML',
        'google-generativeai',
        'pillow',
        'numpy',
        'SpeechRecognition',
        'pyaudio'
    ],
    zip_safe=True,
    maintainer='het',
    maintainer_email='het915@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_controller = t1_keyboard_control.keyboard_controller:main',
            'keyboard_controller_websocket = t1_keyboard_control.keyboard_controller_websocket:main',
            'camera_viewer = t1_keyboard_control.camera_viewer:main',
            'camera_demo = t1_keyboard_control.camera_demo:main',
            'k1_camera_streamer = t1_keyboard_control.k1_camera_streamer:main',
            'test_camera_publisher = t1_keyboard_control.test_camera_publisher:main',
            'usb_camera_publisher = t1_keyboard_control.usb_camera_publisher:main',
            'k1_camera_agent = t1_keyboard_control.k1_camera_agent:main',
            'gemini_robot_controller = t1_keyboard_control.gemini_robot_controller:main',
            'image_converter = t1_keyboard_control.image_converter:main',
            'cmd_vel_bridge = t1_keyboard_control.cmd_vel_bridge:main',
            'voice_command_publisher = t1_keyboard_control.voice_command_publisher:main',
            'simple_robot_commander = t1_keyboard_control.simple_robot_commander:main',
        ],
    },
)
