// some function about alignment
// This part is moved from rpg_SVO with modification to support VILL
#ifndef ALIGN_H_
#define ALIGN_H_

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <stdint.h>
#include <math.h> 

using namespace Eigen;

namespace vill {

    /**
     * @brief align a pixel with reference image patch
     * @param[in] cur_img The current image
     * @param[in] ref_patch_with_boarder the patch with boarder, used to compute the gradient (or FEJ)
     * @param[in] ref_patch the patch in reference frame, by default is 64x64
     * @param[in] n_iter maximum iterations
     * @param[out] cur_px_estimate the estimated position in current image, must have an initial value
     * @return True if successful
     */
    bool Align2D(
            const cv::Mat &cur_img,
            uint8_t *ref_patch_with_border,
            uint8_t *ref_patch,
            const int n_iter,
            Vector2f &cur_px_estimate,
            bool no_simd = false);

}

#endif
