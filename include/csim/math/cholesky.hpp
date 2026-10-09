#pragma once

#include <csim/math/decomposition_detail.hpp>

namespace csim::math {
class NotPositiveDefiniteError : public std::domain_error {
public:
    using std::domain_error::domain_error;
};
template<std::size_t Size>
class Cholesky {
public:
    static constexpr double defaultTolerance() { return Size*std::numeric_limits<double>::epsilon(); }
    explicit Cholesky(const Matrix<Size,Size>& matrix,double tolerance=defaultTolerance())
        :scale_(decomposition_detail::scale(matrix)) {
        decomposition_detail::validateTolerance(tolerance);
        decomposition_detail::symmetric(matrix,scale_);
        if (scale_==0) throw NotPositiveDefiniteError("Cholesky requires positive definiteness");
        for (std::size_t row=0;row<Size;++row) {
            for (std::size_t col=0;col<=row;++col) {
                double value=matrix(row,col)/scale_;
                for (std::size_t previous=0;previous<col;++previous) value-=lower_(row,previous)*lower_(col,previous);
                if (row==col) {
                    if (value<=tolerance) throw NotPositiveDefiniteError("Cholesky pivot is not positive or below threshold");
                    lower_(row,col)=std::sqrt(value);
                } else lower_(row,col)=decomposition_detail::finite(value/lower_(col,col));
            }
        }
    }
    Matrix<Size,Size> lower() const { return lower_*std::sqrt(scale_); }
    template<std::size_t RhsCols>
    Matrix<Size,RhsCols> solve(const Matrix<Size,RhsCols>& rhs) const {
        if (!rhs.isFinite()) throw std::invalid_argument("Cholesky requires finite right-hand side");
        Matrix<Size,RhsCols> result;
        for (std::size_t col=0;col<RhsCols;++col) {
            for (std::size_t row=0;row<Size;++row) {
                double value=rhs(row,col)/scale_;
                for (std::size_t previous=0;previous<row;++previous) value-=lower_(row,previous)*result(previous,col);
                result(row,col)=decomposition_detail::finite(value/lower_(row,row));
            }
            for (std::size_t remaining=Size;remaining>0;--remaining) {
                const auto row=remaining-1;
                double value=result(row,col);
                for (std::size_t next=row+1;next<Size;++next) value-=lower_(next,row)*result(next,col);
                result(row,col)=decomposition_detail::finite(value/lower_(row,row));
            }
        }
        return result;
    }
    template<std::size_t Dimension=Size>
    std::enable_if_t<Dimension==3 && Dimension==Size,Vector3> solve(const Vector3& rhs) const {
        const auto result=solve(Matrix<3,1>{rhs.x,rhs.y,rhs.z});
        return {result(0,0),result(1,0),result(2,0)};
    }
private:
    Matrix<Size,Size> lower_;
    double scale_;
};
}
