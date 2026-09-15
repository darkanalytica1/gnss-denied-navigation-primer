"""Just enough dense linear algebra for small filters, in plain Python.

Matrices are lists of row lists. Sizes here are 2x2 to 4x4, so clarity wins
over speed and the package keeps zero runtime dependencies.
"""
from __future__ import annotations

from typing import List, Sequence

Matrix = List[List[float]]
Vector = List[float]


def eye(n: int, scale: float = 1.0) -> Matrix:
    return [[scale if i == j else 0.0 for j in range(n)] for i in range(n)]


def diag(values: Sequence[float]) -> Matrix:
    n = len(values)
    return [[float(values[i]) if i == j else 0.0 for j in range(n)] for i in range(n)]


def transpose(a: Matrix) -> Matrix:
    return [list(row) for row in zip(*a)]


def add(a: Matrix, b: Matrix) -> Matrix:
    return [[x + y for x, y in zip(ra, rb)] for ra, rb in zip(a, b)]


def sub(a: Matrix, b: Matrix) -> Matrix:
    return [[x - y for x, y in zip(ra, rb)] for ra, rb in zip(a, b)]


def mul(a: Matrix, b: Matrix) -> Matrix:
    bt = transpose(b)
    return [[sum(x * y for x, y in zip(row, col)) for col in bt] for row in a]


def mulv(a: Matrix, v: Sequence[float]) -> Vector:
    return [sum(x * y for x, y in zip(row, v)) for row in a]


def vsub(a: Sequence[float], b: Sequence[float]) -> Vector:
    return [x - y for x, y in zip(a, b)]


def vadd(a: Sequence[float], b: Sequence[float]) -> Vector:
    return [x + y for x, y in zip(a, b)]


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def symmetrize(a: Matrix) -> Matrix:
    n = len(a)
    return [[0.5 * (a[i][j] + a[j][i]) for j in range(n)] for i in range(n)]


def inv(a: Matrix) -> Matrix:
    """Gauss-Jordan inverse with partial pivoting."""
    n = len(a)
    m = [list(map(float, row)) + [1.0 if i == j else 0.0 for j in range(n)] for i, row in enumerate(a)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-15:
            raise ValueError("matrix is singular")
        m[col], m[pivot] = m[pivot], m[col]
        p = m[col][col]
        m[col] = [x / p for x in m[col]]
        for r in range(n):
            if r != col and m[r][col] != 0.0:
                f = m[r][col]
                m[r] = [x - f * y for x, y in zip(m[r], m[col])]
    return [row[n:] for row in m]


def quad_form(v: Sequence[float], a: Matrix) -> float:
    """v^T A v."""
    return dot(v, mulv(a, v))
