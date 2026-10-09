#pragma once

#include <csim/math/decomposition_detail.hpp>

namespace csim::math {
template<std::size_t Size>
class PivotedLDLT {
public:
    static constexpr double defaultTolerance() { return Size*std::numeric_limits<double>::epsilon(); }
    explicit PivotedLDLT(const Matrix<Size,Size>& matrix,double tolerance=defaultTolerance())
        :scale_(decomposition_detail::scale(matrix)) {
        decomposition_detail::validateTolerance(tolerance);
        decomposition_detail::symmetric(matrix,scale_);
        if (scale_==0) throw SingularMatrixError("LDLT cannot factor a zero matrix");
        auto remaining_matrix=matrix/scale_;
        for (std::size_t index=0;index<Size;++index) permutation_[index]=index;
        auto swap_index=[&](std::size_t first,std::size_t second,std::size_t completed) {
            if (first==second) return;
            for (std::size_t col=0;col<Size;++col) std::swap(remaining_matrix(first,col),remaining_matrix(second,col));
            for (std::size_t row=0;row<Size;++row) std::swap(remaining_matrix(row,first),remaining_matrix(row,second));
            for (std::size_t col=0;col<completed;++col) std::swap(lower_(first,col),lower_(second,col));
            std::swap(permutation_[first],permutation_[second]);
        };
        std::size_t stage=0;
        while (stage<Size) {
            double diagonal_max=0,off_diagonal_max=0;
            std::size_t diagonal_index=stage,first_index=stage,second_index=stage;
            for (std::size_t row=stage;row<Size;++row) {
                if (std::abs(remaining_matrix(row,row))>diagonal_max) {
                    diagonal_max=std::abs(remaining_matrix(row,row)); diagonal_index=row;
                }
                for (std::size_t col=stage;col<row;++col) {
                    if (std::abs(remaining_matrix(row,col))>off_diagonal_max) {
                        off_diagonal_max=std::abs(remaining_matrix(row,col)); first_index=col; second_index=row;
                    }
                }
            }
            if (std::max(diagonal_max,off_diagonal_max)<=tolerance)
                throw SingularMatrixError("LDLT pivot below relative threshold");
            if (diagonal_max>=.64*off_diagonal_max) {
                swap_index(stage,diagonal_index,stage);
                blocks_[stage]=1;
                const double pivot=remaining_matrix(stage,stage);
                if (std::abs(pivot)<=tolerance) throw SingularMatrixError("LDLT diagonal pivot below relative threshold");
                diagonal_(stage,stage)=pivot;
                for (std::size_t row=stage+1;row<Size;++row)
                    lower_(row,stage)=decomposition_detail::finite(remaining_matrix(row,stage)/pivot);
                for (std::size_t row=stage+1;row<Size;++row) {
                    for (std::size_t col=stage+1;col<=row;++col) {
                        const double value=decomposition_detail::finite(remaining_matrix(row,col)
                            -lower_(row,stage)*remaining_matrix(col,stage));
                        remaining_matrix(row,col)=value; remaining_matrix(col,row)=value;
                    }
                }
                ++stage;
            } else {
                swap_index(stage,first_index,stage);
                swap_index(stage+1,second_index,stage);
                blocks_[stage]=2;
                const double first=remaining_matrix(stage,stage),second=remaining_matrix(stage+1,stage+1);
                const double cross=remaining_matrix(stage+1,stage),block_scale=std::abs(cross);
                const double normalized_first=first/block_scale,normalized_second=second/block_scale;
                const double normalized_cross=cross/block_scale;
                const double determinant=normalized_first*normalized_second-1;
                diagonal_(stage,stage)=first; diagonal_(stage+1,stage+1)=second;
                diagonal_(stage,stage+1)=cross; diagonal_(stage+1,stage)=cross;
                for (std::size_t row=stage+2;row<Size;++row) {
                    const double left=remaining_matrix(row,stage)/block_scale,right=remaining_matrix(row,stage+1)/block_scale;
                    lower_(row,stage)=decomposition_detail::finite((left*normalized_second-right*normalized_cross)/determinant);
                    lower_(row,stage+1)=decomposition_detail::finite((right*normalized_first-left*normalized_cross)/determinant);
                }
                for (std::size_t row=stage+2;row<Size;++row) {
                    for (std::size_t col=stage+2;col<=row;++col) {
                        const double value=decomposition_detail::finite(remaining_matrix(row,col)
                            -lower_(row,stage)*remaining_matrix(col,stage)-lower_(row,stage+1)*remaining_matrix(col,stage+1));
                        remaining_matrix(row,col)=value; remaining_matrix(col,row)=value;
                    }
                }
                stage+=2;
            }
        }
    }
    const Matrix<Size,Size>& lower() const noexcept { return lower_; }
    Matrix<Size,Size> diagonal() const {
        const auto result=diagonal_*scale_;
        if (!result.isFinite()) throw std::overflow_error("LDLT diagonal factor overflow");
        return result;
    }
    const std::array<std::size_t,Size>& permutation() const noexcept { return permutation_; }
    const std::array<std::size_t,Size>& blockSizes() const noexcept { return blocks_; }
    Matrix<Size,Size> permutationMatrix() const {
        Matrix<Size,Size> result;
        for (std::size_t row=0;row<Size;++row) result(row,permutation_[row])=1;
        return result;
    }
    template<std::size_t RhsCols>
    Matrix<Size,RhsCols> solve(const Matrix<Size,RhsCols>& rhs) const {
        if (!rhs.isFinite()) throw std::invalid_argument("LDLT requires finite right-hand side");
        Matrix<Size,RhsCols> temporary,result;
        for (std::size_t col=0;col<RhsCols;++col) {
            for (std::size_t row=0;row<Size;++row) {
                double value=rhs(permutation_[row],col)/scale_;
                for (std::size_t previous=0;previous<row;++previous) value-=lower_(row,previous)*temporary(previous,col);
                temporary(row,col)=decomposition_detail::finite(value);
            }
            for (std::size_t row=0;row<Size;) {
                if (blocks_[row]==1) {
                    temporary(row,col)=decomposition_detail::finite(temporary(row,col)/diagonal_(row,row));
                    ++row;
                } else {
                    const double block_scale=std::abs(diagonal_(row,row+1));
                    const double first=diagonal_(row,row)/block_scale,second=diagonal_(row+1,row+1)/block_scale;
                    const double cross=diagonal_(row,row+1)/block_scale,determinant=first*second-1;
                    const double left=temporary(row,col)/block_scale,right=temporary(row+1,col)/block_scale;
                    temporary(row,col)=decomposition_detail::finite((left*second-right*cross)/determinant);
                    temporary(row+1,col)=decomposition_detail::finite((right*first-left*cross)/determinant);
                    row+=2;
                }
            }
            for (std::size_t remaining=Size;remaining>0;--remaining) {
                const auto row=remaining-1;
                double value=temporary(row,col);
                for (std::size_t next=row+1;next<Size;++next) value-=lower_(next,row)*result(permutation_[next],col);
                result(permutation_[row],col)=decomposition_detail::finite(value);
            }
        }
        return result;
    }
private:
    Matrix<Size,Size> lower_=Matrix<Size,Size>::identity(),diagonal_;
    std::array<std::size_t,Size> permutation_{},blocks_{};
    double scale_;
};
}
