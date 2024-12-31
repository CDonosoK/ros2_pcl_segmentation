from setuptools import setup

package_name = 'pcl_human_segmentation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='cdonoso',
    maintainer_email='clemente.donosok@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = pcl_human_segmentation.camera:main',
            'human_detection = pcl_human_segmentation.human_detection:main',
        ],
    },
)
