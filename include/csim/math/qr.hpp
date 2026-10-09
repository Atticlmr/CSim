#pragma once

#include <csim/math/decomposition_detail.hpp>

namespace csim::math {
template<std::size_t Rows,std::size_t Cols>
class HouseholderQR {
public:
    static constexpr std::size_t diagonal_size=std::min(Rows,Cols);
    static constexpr double defaultTolerance() { return std::max(Rows,Cols)*std::numeric_limits<double>::epsilon(); }
    explicit HouseholderQR(const Matrix<Rows,Cols>& matrix,double tolerance=defaultTolerance())
        :scale_(decomposition_detail::scale(matrix)),tolerance_(tolerance) {
        decomposition_detail::validateTolerance(tolerance);
        for (std::size_t col=0;col<Cols;++col) permutation_[col]=col;
        if (scale_==0) return;
        upper_=matrix/scale_;
        for (std::size_t stage=0;stage<diagonal_size;++stage) {
            std::size_t pivot=stage;
            double largest_norm=-1;
            for (std::size_t col=stage;col<Cols;++col) {
                double norm=0;
                for (std::size_t row=stage;row<Rows;++row) norm=std::hypot(norm,upper_(row,col));
                if (norm>largest_norm) { largest_norm=norm; pivot=col; }
            }
            if (largest_norm==0) break;
            if (pivot!=stage) {
                for (std::size_t row=0;row<Rows;++row) std::swap(upper_(row,stage),upper_(row,pivot));
                std::swap(permutation_[stage],permutation_[pivot]);
            }
            std::array<double,Rows> reflector{};
            for (std::size_t row=stage;row<Rows;++row) reflector[row]=upper_(row,stage)/largest_norm;
            reflector[stage]+=std::copysign(1.,reflector[stage]);
            double reflector_norm=0;
            for (std::size_t row=stage;row<Rows;++row) reflector_norm=std::hypot(reflector_norm,reflector[row]);
            for (std::size_t row=stage;row<Rows;++row) reflector[row]/=reflector_norm;
            for (std::size_t col=stage;col<Cols;++col) {
                double dot=0;
                for (std::size_t row=stage;row<Rows;++row) dot+=reflector[row]*upper_(row,col);
                for (std::size_t row=stage;row<Rows;++row) upper_(row,col)-=2*reflector[row]*dot;
            }
            for (std::size_t row=0;row<Rows;++row) {
                double dot=0;
                for (std::size_t col=stage;col<Rows;++col) dot+=orthogonal_(row,col)*reflector[col];
                for (std::size_t col=stage;col<Rows;++col) orthogonal_(row,col)-=2*dot*reflector[col];
            }
            for (std::size_t row=stage+1;row<Rows;++row) upper_(row,stage)=0;
        }
        double largest_diagonal=0;
        for (std::size_t diagonal=0;diagonal<diagonal_size;++diagonal)
            largest_diagonal=std::max(largest_diagonal,std::abs(upper_(diagonal,diagonal)));
        for (std::size_t diagonal=0;diagonal<diagonal_size;++diagonal)
            if (std::abs(upper_(diagonal,diagonal))>tolerance_*largest_diagonal) ++rank_;
    }
    const Matrix<Rows,Rows>& orthogonal() const noexcept { return orthogonal_; }
    Matrix<Rows,Cols> upper() const {
        const auto result=upper_*scale_;
        if (!result.isFinite()) throw std::overflow_error("QR upper factor overflow");
        return result;
    }
    const std::array<std::size_t,Cols>& permutation() const noexcept { return permutation_; }
    Matrix<Cols,Cols> permutationMatrix() const {
        Matrix<Cols,Cols> result;
        for (std::size_t col=0;col<Cols;++col) result(permutation_[col],col)=1;
        return result;
    }
    std::size_t rank() const noexcept { return rank_; }
    template<std::size_t RhsCols>
    Matrix<Cols,RhsCols> solve(const Matrix<Rows,RhsCols>& rhs) const {
        if (!rhs.isFinite()) throw std::invalid_argument("QR requires finite right-hand side");
        if constexpr (Rows<Cols) throw SingularMatrixError("Use SVD for underdetermined systems");
        if (rank_!=Cols) throw SingularMatrixError("QR least-squares solve requires full column rank");
        const auto transformed=orthogonal_.transposed()*(rhs/scale_);
        Matrix<Cols,RhsCols> result;
        for (std::size_t col=0;col<RhsCols;++col) {
            for (std::size_t remaining=Cols;remaining>0;--remaining) {
                const auto row=remaining-1;
                double value=transformed(row,col);
                for (std::size_t next=row+1;next<Cols;++next) value-=upper_(row,next)*result(permutation_[next],col);
                result(permutation_[row],col)=decomposition_detail::finite(value/upper_(row,row));
            }
        }
        return result;
    }
private:
    Matrix<Rows,Rows> orthogonal_=Matrix<Rows,Rows>::identity();
    Matrix<Rows,Cols> upper_;
    std::array<std::size_t,Cols> permutation_{};
    double scale_,tolerance_;
    std::size_t rank_=0;
};
}
