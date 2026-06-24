import os
from collections import defaultdict
from pathlib import Path

from distutils.command.install_data import install_data
from setuptools import find_packages, setup

try:
    from colcon_core.distutils.commands.symlink_data import symlink_data
except ImportError:
    symlink_data = install_data


class ForceSymlinkData(symlink_data):
    def copy_file(self, src, dst, **kwargs):
        target = os.path.join(dst, os.path.basename(src)) if os.path.isdir(dst) else dst
        if os.path.isfile(target) or os.path.islink(target):
            os.remove(target)
        return super().copy_file(src, dst, **kwargs)


package_name = 'yolo_realsense'


def collect_model_data_files():
    grouped_files = defaultdict(list)
    models_dir = Path('models')
    if not models_dir.is_dir():
        return []

    for path in sorted(p for p in models_dir.rglob('*') if p.is_file()):
        destination = os.path.join('share', package_name, path.parent.as_posix())
        grouped_files[destination].append(str(path))

    return sorted(grouped_files.items())

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + collect_model_data_files(),
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'ultralytics',
    ],
    zip_safe=True,
    maintainer='frlab',
    maintainer_email='frlab@todo.todo',
    description='RealSense YOLO segmentation and thin-part grasp trigger nodes.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_realsense.yolo_publisher_node:main',
            'yolo_seg_node = yolo_realsense.yolo26_seg_mask_publisher_node:main',
            'thin_part_grasp_node = yolo_realsense.thin_part_grasp_trigger_node:main',
            'thin_part_trigger_client = yolo_realsense.keyboard_trigger_client:main',
            'yolo_seg_viewer = yolo_realsense.realsense_yolo26_seg_viewer:main',
        ],
    },
    cmdclass={
        'symlink_data': ForceSymlinkData,
    },
)
