"""Four-rotor bounded least squares, implemented entirely in Python."""
import itertools
import math


def multiply(matrix, vector):
    return [sum(a*b for a,b in zip(row, vector)) for row in matrix]


def solve(matrix, rhs):
    """Small square system with partial pivoting; reject singular geometry."""
    a=[list(row)+[value] for row,value in zip(matrix,rhs)]
    n=len(a); scale=max(abs(x) for row in matrix for x in row)
    for k in range(n):
        pivot=max(range(k,n),key=lambda i:abs(a[i][k]))
        if abs(a[pivot][k])<=1e-12*scale: raise ValueError('Singular rotor geometry')
        a[k],a[pivot]=a[pivot],a[k]
        for i in range(k+1,n):
            ratio=a[i][k]/a[k][k]
            for j in range(k,n+1): a[i][j]-=ratio*a[k][j]
    x=[0.]*n
    for i in reversed(range(n)):
        x[i]=(a[i][n]-sum(a[i][j]*x[j] for j in range(i+1,n)))/a[i][i]
    if not all(math.isfinite(v) for v in x): raise OverflowError('Rotor solve overflow')
    return x


def bounded_allocate(matrix, desired, weights, upper):
    """Minimize ||diag(weights)(A*f-desired)||², 0 <= f <= upper.

    Caller validates 4x4 nonsingular geometry and finite inputs. Enumerate the
    81 box faces and solve each free least-squares problem by Givens rotations.
    """
    exact=solve(matrix,desired)
    if all(0<=f<=u for f,u in zip(exact,upper)): return exact
    weights=[w/max(weights) for w in weights]
    best=None; best_score=math.inf
    for face in itertools.product(range(3),repeat=4):
        if any(c==2 and not math.isfinite(u) for c,u in zip(face,upper)): continue
        x=[u if c==2 else 0. for c,u in zip(face,upper)]
        free=[i for i,c in enumerate(face) if c==0]; n=len(free)
        a=[[row[j]*w for j in free] for row,w in zip(matrix,weights)]
        rhs=[(b-sum(v*f for v,f in zip(row,x)))*w for row,b,w in zip(matrix,desired,weights)]
        for col in range(n):
            for row in range(3,col,-1):
                radius=math.hypot(a[col][col],a[row][col])
                if not radius: continue
                c=a[col][col]/radius; s=a[row][col]/radius
                for j in range(col,n):
                    a[col][j],a[row][j]=c*a[col][j]+s*a[row][j],-s*a[col][j]+c*a[row][j]
                rhs[col],rhs[row]=c*rhs[col]+s*rhs[row],-s*rhs[col]+c*rhs[row]
        valid=True
        for row in reversed(range(n)):
            if not a[row][row]: raise ValueError('Degenerate allocator face')
            value=(rhs[row]-sum(a[row][j]*x[free[j]] for j in range(row+1,n)))/a[row][row]
            if not math.isfinite(value): raise OverflowError('Allocator overflow')
            tolerance=1e-11*max(1.,abs(value))
            if value < -tolerance or value > upper[free[row]]+tolerance:
                valid=False; break
            x[free[row]]=max(0.,min(value,upper[free[row]]))
        if not valid: continue
        score=sum((w*(v-b))**2 for w,v,b in zip(weights,multiply(matrix,x),desired))
        if not math.isfinite(score): raise OverflowError('Allocator objective overflow')
        if score<best_score: best_score=score; best=x
    if best is None: raise ValueError('No feasible rotor allocation')
    return best
