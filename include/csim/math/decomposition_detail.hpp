#pragma once

#include <csim/math/lu.hpp>

namespace csim::math::decomposition_detail {
inline double finite(double value) {
    if (!std::isfinite(value)) throw std::overflow_error("Matrix decomposition arithmetic overflow");
    return value;
}
inline void validateTolerance(double tolerance) {
    if (!std::isfinite(tolerance) || tolerance<0 || tolerance>=1)
        throw std::invalid_argument("Relative decomposition tolerance must be finite and in [0, 1)");
}
template<std::size_t Rows,std::size_t Cols>
double scale(const Matrix<Rows,Cols>& matrix) {
    if (!matrix.isFinite()) throw std::invalid_argument("Matrix decomposition requires finite values");
    double result=0;
    for (std::size_t row=0;row<Rows;++row)
        for (std::size_t col=0;col<Cols;++col) result=std::max(result,std::abs(matrix(row,col)));
    return result;
}
template<std::size_t Size>
void symmetric(const Matrix<Size,Size>& matrix,double scale) {
    if (scale==0) return;
    const double tolerance=64*Size*std::numeric_limits<double>::epsilon();
    for (std::size_t row=0;row<Size;++row)
        for (std::size_t col=0;col<row;++col)
            if (std::abs(matrix(row,col)/scale-matrix(col,row)/scale)>tolerance)
                throw std::invalid_argument("Matrix decomposition requires a symmetric matrix");
}
}
