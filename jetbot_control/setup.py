from setuptools import setup
import os
from glob import glob

package_name = 'jetbot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # Wajib untuk ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sandi',
    maintainer_email='mahasiswa@email.com',
    description='Motion Control JetBot KBBC dan KBBC+PID',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ── ARSITEKTUR BARU (gunakan ini) ──────────────────
            # Satu node menggabungkan semua logika simulasi.
            # Jalankan dengan: ros2 launch jetbot_control master.launch.py
            'master_node      = jetbot_control.master_node:main',

            # ── ARSITEKTUR LAMA (referensi, tidak digunakan) ───
            'trajectory_node  = jetbot_control.trajectory_node:main',
            'plant_node       = jetbot_control.plant_node:main',
            'kbbc_node        = jetbot_control.kbbc_node:main',
            'pid_node         = jetbot_control.pid_node:main',
            'sensor_node      = jetbot_control.sensor_node:main',
            'logger_node      = jetbot_control.logger_node:main',
            'labview_bridge   = jetbot_control.labview_bridge:main',
        ],
    },
)
