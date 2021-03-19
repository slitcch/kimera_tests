# WIP. Might be too much trouble to get this working.
project(
	't26x-kimera',
	['c', 'cpp'],
	version: '0',
	license: 'BSL-1.0',
	meson_version: '>=0.49.0',
	default_options: [
		'c_std=c11',
		'warning_level=3',
	],
)

cc = meson.get_compiler('c')
cpp = meson.get_compiler('cpp')

add_project_arguments(cc.get_supported_arguments([
	'-D_XOPEN_SOURCE=700',
	'-Wno-unused-parameter',
	'-Werror-implicit-function-declaration',
	'-Werror=incompatible-pointer-types'
]), language: 'c')

add_project_arguments(cpp.get_supported_arguments([
	'-D_XOPEN_SOURCE=700',
	'-Wno-unused-parameter',
	'-Wno-deprecated-copy', # Eigen
	'-Wno-c11-extensions' # clang OpenCV
]), language: 'cpp')


rs       = dependency('realsense2', required: true)

dependency('OpenCV' version : '>=4' , required: true)
dependency('Eigen3' , required: true)
dependency('Gflags' , required: true)
dependency('Glog' version : '>=0.3.5' , required: true)
dependency('GTSAM' version : '>=4.0.2' , required: true)
dependency('opengv' , required: true)
dependency('DBoW2' , required: true)