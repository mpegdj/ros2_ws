from setuptools import setup
import os
from glob import glob

package_name = 'my_map'

# 모든 하위 폴더의 파일을 자동으로 수집하는 함수
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            # 설치될 경로: share/패키지명/원본폴더경로
            install_path = os.path.join('share', package_name, path)
            # 원본 파일 경로
            file_path = os.path.join(path, filename)
            paths.append((install_path, [file_path]))
    return paths

# data_files 기본 리스트 + Nav2 추가 
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # Launch 파일들 (py, xml, yaml 모두 포함 가능)
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    # 월드 파일
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    # 지도 파일 (.yaml, .pgm)
    (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    # Nav2 파라미터 파일 (.yaml)
    (os.path.join('share', package_name, 'params'), glob('params/*')),
]

# models 폴더의 파일들을 재귀적으로 추가
data_files.extend(package_files('models'))

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=data_files, # 합쳐진 리스트 사용
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dtv',
    maintainer_email='dtv@todo.todo',
    description='My first gazebo world',
    license='Apache-2.0',
)
