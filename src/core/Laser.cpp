#include "core/Laser.h"
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace mllm
{
LaserLine extractLaser(const cv::Mat& image, const LaserConfig& c)
{
    if (image.empty() || image.depth() != CV_8U || (image.channels() != 1 && image.channels() != 3))
        throw std::runtime_error("提线需要 8 位灰度或 BGR 图像");
    if (c.threshold < 0 || c.threshold > 255 || !std::isfinite(c.width) || c.width < 0.5 || c.width > 30 ||
        c.channel < 0 || c.channel > 3 || c.method < 0 || c.method > 1)
        throw std::runtime_error("提线参数无效");
    cv::Rect roi = c.roi;
    if (roi.width == 0)
        roi.width = image.cols - roi.x;
    if (roi.height == 0)
        roi.height = image.rows - roi.y;
    if (roi.x < 0 || roi.y < 0 || roi.width <= 0 || roi.height <= 0 ||
        (roi & cv::Rect(0, 0, image.cols, image.rows)) != roi)
        throw std::runtime_error("ROI 超出图像范围");
    cv::Mat gray;
    if (image.channels() == 1)
        gray = image(roi).clone();
    else if (c.channel == 0)
        cv::cvtColor(image(roi), gray, cv::COLOR_BGR2GRAY);
    else
        cv::extractChannel(image(roi), gray, 3 - c.channel);
    // Match UWScan's row/column formulation by transposing horizontal stripes.
    if (c.horizontal)
        cv::transpose(gray, gray);
    std::vector<cv::Point2d> centers;
    if (c.method == 0)
    {
        for (int y = 0; y < gray.rows; ++y)
        {
            const auto* row = gray.ptr<uchar>(y);
            // Select the brightest contiguous segment; disconnected reflections must
            // not create a centroid in the dark gap between two stripes.
            double bestSum = 0;
            int bestStart = 0, bestEnd = 0;
            for (int x = 0; x < gray.cols;)
            {
                if (row[x] <= c.threshold)
                {
                    ++x;
                    continue;
                }
                const int start = x;
                double sum = 0;
                while (x < gray.cols && row[x] > c.threshold)
                {
                    sum += row[x];
                    ++x;
                }
                if (sum > bestSum)
                {
                    bestSum = sum;
                    bestStart = start;
                    bestEnd = x - 1;
                }
            }
            if (bestSum > 0)
            {
                // Use the threshold for detection; retaining background-corrected
                // Gaussian tails avoids truncation bias in subpixel centroids.
                const int pad = static_cast<int>(std::ceil(2 * c.width));
                int lo = bestStart, hi = bestEnd;
                while (lo > std::max(0, bestStart - pad) && row[lo - 1] <= c.threshold)
                    --lo;
                while (hi < std::min(gray.cols - 1, bestEnd + pad) && row[hi + 1] <= c.threshold)
                    ++hi;
                const double background = std::min(row[lo], row[hi]);
                double sum = 0, weighted = 0;
                for (int x = lo; x <= hi; ++x)
                {
                    const double weight = std::max(0.0, row[x] - background);
                    sum += weight;
                    weighted += x * weight;
                }
                if (sum > 0)
                    centers.emplace_back(weighted / sum, y);
            }
        }
    }
    else
    {
        // UWScan2's Gaussian derivatives / Hessian normal and Taylor subpixel
        // offset, with stable eigenvectors for axis-aligned lines.
        const double sigma = c.width / std::sqrt(3.0);
        const int radius = std::max(1, static_cast<int>(std::ceil(3 * sigma))), size = 2 * radius + 1;
        cv::Mat gx(size, size, CV_64F), gy = gx.clone(), gxx = gx.clone(), gxy = gx.clone(), gyy = gx.clone();
        for (int y = -radius; y <= radius; ++y)
            for (int x = -radius; x <= radius; ++x)
            {
                const double g =
                    std::exp(-(x * x + y * y) / (2 * sigma * sigma)) / (2 * CV_PI * sigma * sigma);
                gx.at<double>(y + radius, x + radius) = x * g / (sigma * sigma);
                gy.at<double>(y + radius, x + radius) = y * g / (sigma * sigma);
                gxx.at<double>(y + radius, x + radius) = (x * x / (sigma * sigma) - 1) * g / (sigma * sigma);
                gxy.at<double>(y + radius, x + radius) = x * y * g / std::pow(sigma, 4);
                gyy.at<double>(y + radius, x + radius) = (y * y / (sigma * sigma) - 1) * g / (sigma * sigma);
            }
        // Zero DC response prevents a uniform bright background from becoming
        // a false ridge through truncation of the Gaussian derivative kernels.
        gxx -= cv::mean(gxx)[0];
        gyy -= cv::mean(gyy)[0];
        cv::Mat dx, dy, dxx, dxy, dyy;
        cv::filter2D(gray, dx, CV_64F, gx);
        cv::filter2D(gray, dy, CV_64F, gy);
        cv::filter2D(gray, dxx, CV_64F, gxx);
        cv::filter2D(gray, dxy, CV_64F, gxy);
        cv::filter2D(gray, dyy, CV_64F, gyy);
        for (int y = 0; y < gray.rows; ++y)
        {
            double best = 0;
            cv::Point2d point;
            for (int x = radius; x < gray.cols - radius; ++x)
            {
                if (gray.at<uchar>(y, x) <= c.threshold)
                    continue;
                const double a = dxx.at<double>(y, x), b = dxy.at<double>(y, x), d = dyy.at<double>(y, x);
                const double root = std::hypot(a - d, 2 * b);
                const double l1 = (a + d + root) / 2, l2 = (a + d - root) / 2;
                const double lambda = std::abs(l1) > std::abs(l2) ? l1 : l2;
                if (lambda >= -1e-4)
                    continue; // a bright ridge has negative curvature.
                double nx, ny;
                if (std::abs(b) > 1e-12)
                {
                    nx = b;
                    ny = lambda - a;
                }
                else
                {
                    nx = std::abs(a) > std::abs(d) ? 1 : 0;
                    ny = 1 - nx;
                }
                const double norm = std::hypot(nx, ny);
                nx /= norm;
                ny /= norm;
                const double denominator = a * nx * nx + 2 * b * nx * ny + d * ny * ny;
                const double offset = -(dx.at<double>(y, x) * nx + dy.at<double>(y, x) * ny) / denominator;
                if (std::abs(offset * nx) > 0.5 || std::abs(offset * ny) > 0.5)
                    continue;
                if (-lambda > best)
                {
                    best = -lambda;
                    point = {x + offset * nx, y + offset * ny};
                }
            }
            if (best > 0)
                centers.push_back(point);
        }
    }
    LaserLine result;
    if (image.channels() == 1)
        cv::cvtColor(image, result.preview, cv::COLOR_GRAY2BGR);
    else
        result.preview = image.clone();
    for (auto point : centers)
    {
        if (c.horizontal)
            std::swap(point.x, point.y);
        point.x += roi.x;
        point.y += roi.y;
        result.pixels.push_back(point);
        cv::circle(result.preview, point, 1, cv::Scalar(0, 0, 255), -1);
    }
    cv::rectangle(result.preview, roi, cv::Scalar(0, 180, 0), 1);
    return result;
}
} // namespace mllm
