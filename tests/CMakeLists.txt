# Copyright 2013, Thomas Moulard, LAAS-CNRS
#
# This file is part of roboptim-core.
# roboptim-core is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# roboptim-core is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with roboptim-core.  If not, see <http://www.gnu.org/licenses/>.

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# Set the tests model directory.
ADD_DEFINITIONS(
  "-DTEST_MODEL_DIRECTORY=\"${CMAKE_SOURCE_DIRECTORY}/test/models\"")

ADD_EXECUTABLE(one_dof_revolute_x one_dof_revolute_x.cpp)
PKG_CONFIG_USE_DEPENDENCY(one_dof_revolute_x jrl-mal)
PKG_CONFIG_USE_DEPENDENCY(one_dof_revolute_x urdfdom_headers)
TARGET_LINK_LIBRARIES(one_dof_revolute_x jrl-dynamics-urdf)
