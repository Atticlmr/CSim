#pragma once

#include <csim/math/decomposition_detail.hpp>

namespace csim::math {
class DecompositionFailure : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};
template<std::size_t Rows,std::size_t Cols>
class JacobiSVD {
public:
    static constexpr std::size_t thin_size=std::min(Rows,Cols);
    static constexpr double defaultTolerance() { return std::max(Rows,Cols)*std::numeric_limits<double>::epsilon(); }
    explicit JacobiSVD(const Matrix<Rows,Cols>& matrix,double tolerance=defaultTolerance(),std::size_t max_sweeps=100)
        :scale_(decomposition_detail::scale(matrix)),tolerance_(tolerance) {
        decomposition_detail::validateTolerance(tolerance);
        if (!max_sweeps) throw std::invalid_argument("SVD requires a positive sweep budget");
        if constexpr (Rows<Cols) {
            const JacobiSVD<Cols,Rows> transposed(matrix.transposed(),tolerance,max_sweeps);
            left_=transposed.right(); right_=transposed.left(); singular_values_=transposed.singularValues();
            for (std::size_t index=0;index<thin_size;++index)
                normalized_values_[index]=scale_==0 ? 0 : singular_values_[index]/scale_;
            rank_=transposed.rank();
        } else {
            Matrix<Rows,Cols> working=scale_==0 ? matrix : matrix/scale_;
            auto rotations=Matrix<Cols,Cols>::identity();
            bool converged=false;
            const long double orthogonality_tolerance=8*std::numeric_limits<double>::epsilon();
            for (std::size_t sweep=0;sweep<max_sweeps;++sweep) {
                bool changed=false;
                for (std::size_t first=0;first<Cols;++first) {
                    for (std::size_t second=first+1;second<Cols;++second) {
                        long double first_norm=0,second_norm=0,cross=0;
                        for (std::size_t row=0;row<Rows;++row) {
                            const long double left=working(row,first),right=working(row,second);
                            first_norm+=left*left; second_norm+=right*right; cross+=left*right;
                        }
                        if (cross==0 || std::abs(cross)<=orthogonality_tolerance*std::sqrt(first_norm)*std::sqrt(second_norm)) continue;
                        changed=true;
                        const long double ratio=(second_norm-first_norm)/(2*cross);
                        const double tangent=static_cast<double>(std::copysign(1.L,ratio)/(std::abs(ratio)+std::hypot(1.L,ratio)));
                        const double cosine=1/std::hypot(1.,tangent),sine=cosine*tangent;
                        for (std::size_t row=0;row<Rows;++row) {
                            const double left=working(row,first),right=working(row,second);
                            working(row,first)=cosine*left-sine*right; working(row,second)=sine*left+cosine*right;
                        }
                        for (std::size_t row=0;row<Cols;++row) {
                            const double left=rotations(row,first),right=rotations(row,second);
                            rotations(row,first)=cosine*left-sine*right; rotations(row,second)=sine*left+cosine*right;
                        }
                    }
                }
                if (!changed) { converged=true; break; }
            }
            if (!converged) throw DecompositionFailure("Jacobi SVD sweep budget exhausted");
            for (std::size_t col=0;col<Cols;++col)
                for (std::size_t row=0;row<Rows;++row) normalized_values_[col]=std::hypot(normalized_values_[col],working(row,col));
            for (std::size_t col=0;col<Cols;++col) {
                std::size_t largest=col;
                for (std::size_t next=col+1;next<Cols;++next)
                    if (normalized_values_[next]>normalized_values_[largest]) largest=next;
                if (largest!=col) {
                    std::swap(normalized_values_[col],normalized_values_[largest]);
                    for (std::size_t row=0;row<Rows;++row) std::swap(working(row,col),working(row,largest));
                    for (std::size_t row=0;row<Cols;++row) std::swap(rotations(row,col),rotations(row,largest));
                }
                singular_values_[col]=decomposition_detail::finite(normalized_values_[col]*scale_);
                if (normalized_values_[col]>tolerance_*normalized_values_[0]) ++rank_;
                for (std::size_t row=0;row<Cols;++row) right_(row,col)=rotations(row,col);
                if (normalized_values_[col]>0) {
                    for (std::size_t row=0;row<Rows;++row) left_(row,col)=working(row,col)/normalized_values_[col];
                } else {
                    bool completed=false;
                    for (std::size_t axis=0;axis<Rows && !completed;++axis) {
                        std::array<double,Rows> candidate{};
                        candidate[axis]=1;
                        for (int pass=0;pass<2;++pass) {
                            for (std::size_t previous=0;previous<col;++previous) {
                                double dot=0;
                                for (std::size_t row=0;row<Rows;++row) dot+=left_(row,previous)*candidate[row];
                                for (std::size_t row=0;row<Rows;++row) candidate[row]-=dot*left_(row,previous);
                            }
                        }
                        double norm=0;
                        for (double value:candidate) norm=std::hypot(norm,value);
                        if (norm>1e-8) {
                            for (std::size_t row=0;row<Rows;++row) left_(row,col)=candidate[row]/norm;
                            completed=true;
                        }
                    }
                    if (!completed) throw DecompositionFailure("SVD null-space basis construction failed");
                }
            }
        }
    }
    const Matrix<Rows,thin_size>& left() const noexcept { return left_; }
    const Matrix<Cols,thin_size>& right() const noexcept { return right_; }
    const std::array<double,thin_size>& singularValues() const noexcept { return singular_values_; }
    std::size_t rank() const noexcept { return rank_; }
    template<std::size_t RhsCols>
    Matrix<Cols,RhsCols> solve(const Matrix<Rows,RhsCols>& rhs) const {
        if (!rhs.isFinite()) throw std::invalid_argument("SVD requires finite right-hand side");
        Matrix<Cols,RhsCols> result;
        if (!rank_) return result;
        for (std::size_t col=0;col<RhsCols;++col) {
            for (std::size_t singular=0;singular<rank_;++singular) {
                long double dot=0;
                for (std::size_t row=0;row<Rows;++row) dot+=static_cast<long double>(left_(row,singular))*rhs(row,col);
                const long double coefficient=(dot/scale_)/normalized_values_[singular];
                for (std::size_t row=0;row<Cols;++row)
                    result(row,col)=decomposition_detail::finite(static_cast<double>(result(row,col)+coefficient*right_(row,singular)));
            }
        }
        return result;
    }
    Matrix<Cols,Rows> pseudoinverse() const { return solve(Matrix<Rows,Rows>::identity()); }
private:
    Matrix<Rows,thin_size> left_;
    Matrix<Cols,thin_size> right_;
    std::array<double,thin_size> singular_values_{},normalized_values_{};
    double scale_,tolerance_;
    std::size_t rank_=0;
};
}
