// clang-format off

// Copyright (c) 2023, S(apienza) R(obust) R(obotics) G(roup)

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// clang-format on

#pragma once
#include "types.h"

namespace ca2lib {

/**
 * @brief Types of image normalizations:
 * UNCHANGED: Values are stored into the image directly without any
 * normalization
 * NORMALIZE: Rescale all values in range [min, max] to [0, 1]
 * MINMAX: Rescale all values between [a, b] to [0, 1]
 *
 */
enum NormalizationType : uint8_t { UNCHANGED = 0, NORMALIZE = 1, MINMAX = 2 };

/**
 * @brief Computes projection indices for all the points of cloud_in and returns
 * a lookup table. The lookup table contains an index for every pixel. The index
 * may be invalid (-1) or, a positive integer that represents the index of the
 * point in cloud_in that is projected on that pixel
 *
 * The size of the lookup table is automatically infered by the ring field or by
 * the cloud width/height. If these informations are not available, input rows
 * and cols are used otherwise.
 *
 * @param cloud_in Input cloud
 * @param inverse_lut Inverse lookup table: contains coordinate of projection
 *                      for each point
 * @param hfov [Optional] Horizontal Field of View of the LiDAR (assumed 2*pi)
 * @param rows [Optional] Number of rows in the lut table (equal to no. rings)
 * @param cols [Optional] Number of columns in the lut table
 * @return cv::Mat LUT table
 */
cv::Mat projectLidarLUT(const PointCloudXf& cloud_in,
                        std::vector<std::pair<bool, cv::Point2i>>& inverse_lut,
                        const float hfov = 0, unsigned int rows = 0,
                        unsigned int cols = 0);

/**
 * @brief Computes projection indices for all the points of cloud_in and returns
 * a lookup table. The lookup table contains an index for every pixel. The index
 * may be invalid (-1) or, a positive integer that represents the index of the
 * point in cloud_in that is projected on that pixel
 *
 * The size of the lookup table is automatically infered by the ring field or by
 * the cloud width/height. If these informations are not available, input rows
 * and cols are used otherwise.
 *
 * This function is an overload that does not expose the inverse_lut.
 *
 * @param cloud_in Input cloud
 * @param hfov [Optional] Horizontal Field of View of the LiDAR (assumed 2*pi)
 * @param rows [Optional] Number of rows in the lut table (equal to no. rings)
 * @param cols [Optional] Number of columns in the lut table
 * @return cv::Mat LUT table
 */
cv::Mat projectLidarLUT(const PointCloudXf& cloud_in, const float hfov = 0,
                        unsigned int rows = 0, unsigned int cols = 0);

/**
 * @brief Composes an image using Projection by ID of a given cloud's channel.
 *
 * @param cloud_in Input cloud
 * @param lut_table LookUp Table computed with projectLidarLUT
 * @param channel channel to render
 * @param norm_type type of normalization technique
 * @param a first normalization factor
 * @param b second normalization factor
 * @return cv::Mat
 */
cv::Mat composeChannelImage(const PointCloudXf& cloud_in,
                            const cv::Mat& lut_table,
                            const std::string& channel,
                            const NormalizationType norm_type = MINMAX,
                            const float a = 1.0f, const float b = 0.0f);

/**
 * @brief Computes the SPHERICAL projection indices for all the points of
 * cloud_in and returns a lookup table. The lookup table contains an index for
 * every pixel. The index may be invalid (-1) or, a positive integer that
 * represents the index of the point in cloud_in that is projected on that pixel
 *
 * @param cloud_in Input cloud
 * @param hfov Horizontal Field of View of the LiDAR
 * @param vfov Vertical Field of View of the LiDAR
 * @param rows Number of rows in the lut table
 * @param cols Number of columns in the lut table
 * @return cv::Mat LUT table
 */
cv::Mat projectSphericalLidarLUT(const PointCloudXf& cloud_in, const float hfov,
                                 const float vfov, unsigned int rows,
                                 unsigned int cols);

/**
 * @brief Computes the PINHOLE projection indices for all the points of cloud_in
 * and returns a lookup table. The Lookup table contains an index for
 * every pixel. THe index may be invalid (-1) or, a positive integer that
 * represents the index of the point in cloud_in that is projected on that pixel
 *
 * @param cloud_in Input cloud
 * @param inverse_lut Inverse lookup table: contains coordinate of projection
 *                      for each point
 * @param image_size Size of the lookup table
 * @param camera_intrinsics camera intrinsics (K, dist_coeffs)
 * @param camera_T_cloud Isometry that maps points in cloud frame to camera
 * frame (i.e. p_cam = camera_T_cloud * p_cloud)
 * @return cv::Mat
 */
cv::Mat projectPinholeLUT(
    const PointCloudXf& cloud_in,
    std::vector<std::pair<bool, cv::Point2i>>& inverse_lut,
    const cv::Size image_size, const CameraIntrinsics camera_intrinsics,
    const Eigen::Isometry3f& camera_T_cloud);
}  // namespace ca2lib