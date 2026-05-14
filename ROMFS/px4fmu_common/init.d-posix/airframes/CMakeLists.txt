############################################################################
#
#   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_romfs_files(

	4001_gz_x500
    4020_gz_q940_ti
	4002_gz_x500_depth
	4005_gz_x500_vision
	4006_gz_px4vision
	4010_gz_x500_mono_cam
	4011_gz_lawnmower
	4013_gz_x500_lidar_2d
	4014_gz_x500_mono_cam_down
	4016_gz_x500_lidar_down
	4017_gz_x500_lidar_front
	4018_gz_quadtailsitter
	4019_gz_x500_gimbal
	4021_gz_x500_flow

	6011_gazebo-classic_typhoon_h480
	6011_gazebo-classic_typhoon_h480.post

	8011_gz_omnicopter

	10018_gazebo-classic_iris_foggy_lidar
	10019_gazebo-classic_omnicopter
	10030_gazebo-classic_px4vision

	# [22000, 22999] Reserve for custom models
)
