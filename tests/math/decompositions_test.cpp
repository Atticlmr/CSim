#include <csim/math/cholesky.hpp>
#include <csim/math/qr.hpp>
#include <csim/math/ldlt.hpp>
#include <csim/math/svd.hpp>
#include <iostream>
#include <random>

using namespace csim::math;
void check(bool condition) { if (!condition) throw std::runtime_error("Matrix decomposition regression failed"); }
template<std::size_t Rows,std::size_t Cols>
void near(const Matrix<Rows,Cols>& actual,const Matrix<Rows,Cols>& expected,double tolerance=1e-10) {
    for (std::size_t row=0;row<Rows;++row)
        for (std::size_t col=0;col<Cols;++col)
            check(std::isfinite(actual(row,col)) && std::abs(actual(row,col)-expected(row,col))<=tolerance);
}
template<class Error,class Operation>
void rejects(Operation operation) {
    bool rejected=false;
    try { operation(); } catch (const Error&) { rejected=true; }
    check(rejected);
}
template<std::size_t Rows,std::size_t Cols>
void checkSvd(const Matrix<Rows,Cols>& matrix) {
    constexpr auto Size=std::min(Rows,Cols);
    const JacobiSVD<Rows,Cols> solver(matrix);
    Matrix<Size,Size> diagonal;
    for (std::size_t index=0;index<Size;++index) {
        diagonal(index,index)=solver.singularValues()[index];
        check(diagonal(index,index)>=0);
        if (index) check(diagonal(index,index)<=diagonal(index-1,index-1));
    }
    near(solver.left().transposed()*solver.left(),Matrix<Size,Size>::identity());
    near(solver.right().transposed()*solver.right(),Matrix<Size,Size>::identity());
    near(solver.left()*diagonal*solver.right().transposed(),matrix);
    const auto inverse=solver.pseudoinverse();
    near(matrix*inverse*matrix,matrix,1e-8);
    near(inverse*matrix*inverse,inverse,1e-8);
    near((matrix*inverse).transposed(),matrix*inverse,1e-8);
    near((inverse*matrix).transposed(),inverse*matrix,1e-8);
}
int main() {
    try {
        const Matrix<3,3> positive{4,1,2,1,3,0,2,0,5};
        const Matrix<3,2> solution{1,-2,3,4,-1,5};
        for (double scale:{1.,1e-250,1e250}) {
            const Cholesky<3> cholesky(positive*scale);
            near((cholesky.lower()/std::sqrt(scale))*(cholesky.lower()/std::sqrt(scale)).transposed(),positive);
            near(cholesky.solve((positive*solution)*scale),solution);
            const PivotedLDLT<3> ldlt(positive*scale);
            const auto permutation=ldlt.permutationMatrix();
            near(ldlt.lower()*(ldlt.diagonal()/scale)*ldlt.lower().transposed(),permutation*positive*permutation.transposed());
            near(ldlt.solve((positive*solution)*scale),solution);
        }
        check((Cholesky<3>(positive).solve(Vector3{5,10,-3})-Vector3{1,3,-1}).norm()<1e-10);
        rejects<NotPositiveDefiniteError>([] { Cholesky<2> solver(Matrix<2,2>{1,2,2,1}); });
        rejects<NotPositiveDefiniteError>([] { Cholesky<2> solver(Matrix<2,2>{1,0,0,0}); });
        rejects<std::invalid_argument>([] { Cholesky<2> solver(Matrix<2,2>{1,0,1,2}); });
        std::cout<<"PASS: Cholesky reconstruction, multiple RHS, scaling and definiteness checks\n";

        const Matrix<3,3> indefinite{0,0,2,0,3,1,2,1,0};
        const PivotedLDLT<3> ldlt(indefinite);
        const auto permutation=ldlt.permutationMatrix();
        near(ldlt.lower()*ldlt.diagonal()*ldlt.lower().transposed(),permutation*indefinite*permutation.transposed());
        near(ldlt.solve(indefinite*solution),solution);
        const PivotedLDLT<2> exchange(Matrix<2,2>{0,1,1,0});
        check(exchange.blockSizes()[0]==2);
        near(exchange.solve(Matrix<2,1>{2,3}),Matrix<2,1>{3,2});
        rejects<SingularMatrixError>([] { PivotedLDLT<2> solver(Matrix<2,2>{1,1,1,1}); });
        rejects<std::invalid_argument>([] { PivotedLDLT<2> solver(Matrix<2,2>{1,2,1,1}); });
        std::cout<<"PASS: LDLT indefinite pivots, permutations, 2x2 blocks and singular rejection\n";

        const Matrix<4,2> design{1,0,1,1,1,2,1,3};
        const HouseholderQR<4,2> qr(design);
        check(qr.rank()==2);
        near(qr.orthogonal().transposed()*qr.orthogonal(),Matrix<4,4>::identity());
        near(qr.orthogonal()*qr.upper(),design*qr.permutationMatrix());
        const Matrix<4,1> observations{1,2,2,4};
        const auto fit=qr.solve(observations);
        near(design.transposed()*(design*fit-observations),Matrix<2,1>{0,0});
        near(fit,Matrix<2,1>{.9,.9});
        const Matrix<3,2> deficient{1,2,2,4,3,6};
        check(HouseholderQR<3,2>(deficient).rank()==1);
        rejects<SingularMatrixError>([&] { HouseholderQR<3,2>(deficient).solve(Matrix<3,1>{1,2,3}); });
        check(HouseholderQR<2,3>(Matrix<2,3>{1,0,1,0,1,1}).rank()==2);
        std::cout<<"PASS: pivoted Householder QR orthogonality, rank and least-squares optimality\n";

        checkSvd(design); checkSvd(design.transposed()); checkSvd(deficient); checkSvd(deficient.transposed());
        checkSvd(Matrix<3,2>{}); checkSvd(Matrix<1,3>{1,2,3});
        const JacobiSVD<2,2> known_spectrum(Matrix<2,2>{3,1,1,3});
        check(std::abs(known_spectrum.singularValues()[0]-4)<1e-12);
        check(std::abs(known_spectrum.singularValues()[1]-2)<1e-12);
        check(JacobiSVD<3,2>(deficient).rank()==1);
        near(JacobiSVD<3,2>(deficient).solve(Matrix<3,1>{1,2,3}),Matrix<2,1>{.2,.4});
        near(JacobiSVD<2,3>(Matrix<2,3>{1,0,1,0,1,1}).solve(Matrix<2,1>{1,1}),Matrix<3,1>{1./3,1./3,2./3});
        for (double scale:{1e-250,1e250}) {
            near(JacobiSVD<4,2>(design*scale).solve(observations*scale),fit);
            near(HouseholderQR<4,2>(design*scale).solve(observations*scale),fit);
        }
        check(JacobiSVD<2,2>(Matrix<2,2>{1,0,0,1e-14},1e-12).rank()==1);
        rejects<DecompositionFailure>([&] { JacobiSVD<3,3> solver(positive,JacobiSVD<3,3>::defaultTolerance(),1); });
        const auto invalid=Matrix<2,2>{1,0,0,std::numeric_limits<double>::quiet_NaN()};
        rejects<std::invalid_argument>([&] { JacobiSVD<2,2> solver(invalid); });
        rejects<std::invalid_argument>([&] { HouseholderQR<2,2> solver(invalid); });
        rejects<std::invalid_argument>([] { JacobiSVD<2,2> solver(Matrix<2,2>{},-1); });
        rejects<std::invalid_argument>([] { Cholesky<2> solver(Matrix<2,2>::identity(),1); });
        rejects<std::invalid_argument>([] { PivotedLDLT<2> solver(Matrix<2,2>::identity(),-1); });
        rejects<std::invalid_argument>([] { HouseholderQR<2,2> solver(Matrix<2,2>::identity(),1); });
        const Matrix<2,1> invalid_rhs{0,std::numeric_limits<double>::infinity()};
        rejects<std::invalid_argument>([&] { Cholesky<2>(Matrix<2,2>::identity()).solve(invalid_rhs); });
        rejects<std::invalid_argument>([&] { PivotedLDLT<2>(Matrix<2,2>::identity()).solve(invalid_rhs); });
        rejects<std::invalid_argument>([&] { HouseholderQR<2,2>(Matrix<2,2>::identity()).solve(invalid_rhs); });
        rejects<std::invalid_argument>([&] { JacobiSVD<2,2>(Matrix<2,2>::identity()).solve(invalid_rhs); });
        std::cout<<"PASS: Jacobi SVD tall/wide/zero/rank-deficient inputs and Moore-Penrose identities\n";

        std::mt19937 engine(731);
        std::uniform_real_distribution<double> distribution(-1,1);
        for (int sample=0;sample<100;++sample) {
            Matrix<5,5> matrix;
            for (std::size_t row=0;row<5;++row)
                for (std::size_t col=0;col<5;++col) matrix(row,col)=distribution(engine);
            const auto symmetric=matrix+matrix.transposed();
            const PivotedLDLT<5> symmetric_solver(symmetric);
            const auto permuted=symmetric_solver.permutationMatrix();
            near(symmetric_solver.lower()*symmetric_solver.diagonal()*symmetric_solver.lower().transposed(),
                 permuted*symmetric*permuted.transposed());
            near(symmetric*symmetric_solver.solve(Matrix<5,5>::identity()),Matrix<5,5>::identity(),1e-8);
            checkSvd(matrix);
            const HouseholderQR<5,5> square_solver(matrix);
            near(matrix*square_solver.solve(Matrix<5,5>::identity()),Matrix<5,5>::identity(),1e-8);
        }
        std::cout<<"PASS: deterministic random dense reconstruction and solve regressions\n";
        return 0;
    } catch (const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
