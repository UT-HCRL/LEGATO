using FlexIK
using LinearAlgebra
using Test

# Assuming InverseParams and inv function are already defined as shown previously
tol = 1e-3  # Tolerance for floating point comparisons

@testset "Inverse Solver Tests" begin
    params = FlexIK.InverseParams(tolerance=1e-6, damping_coeff=1e-1)

    # Test 1: Invert an identity matrix
    A = I(3)  # 3x3 Identity matrix
    @test FlexIK.srinv(params, A) == A  # Should be equal to itself

    # Test 2: Test with a known matrix and its inverse
    A = [1.0 2.0; 3.0 4.0]
    @test inv(A) ≈ FlexIK.srinv(params, A) atol=tol  # Check if close enough

    # Test 3: Test with a random matrix
    test_size = 50
    for _ in 1:test_size
        m = rand(1:10)
        n = rand(1:10)
        A = rand(m, n)
        if cond(A) < 5e0  # Check if matrix is well-conditioned
            @test pinv(A) ≈ FlexIK.srinv(params, A) atol=tol
        end
    end
end
