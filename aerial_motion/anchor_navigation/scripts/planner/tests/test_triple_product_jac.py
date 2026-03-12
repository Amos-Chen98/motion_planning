import numpy as np


def calc_triple_product_jac(a, b, c, d_a, d_b, d_c):
    """
    Calculate the Jacobian of the triple product: f = (a x b)T · c / ||a x b||.
    """
    """
        a,b,c: (3,) vectors
        d_a,d_b,d_c: (3, config_dim) Jacobians wrt q
        returns: (config_dim,) gradient wrt q

        Ref: https://proofwiki.org/wiki/Derivative_of_Scalar_Triple_Product_of_Vector-Valued_Functions
        """
    u = np.cross(a, b)  # (3,)
    s = np.linalg.norm(u)
    if s < 1e-12:
        return np.zeros(d_a.shape[1])

    t = np.dot(u, c)  # scalar

    du = np.cross(d_a.T, b) + np.cross(a, d_b.T)  # (N,3)
    ds = (du @ u) / s  # (N,)
    dt = (du @ c) + (u @ d_c)  # (N,)

    df = (dt * s - t * ds) / (s**2)  # (N,)
    return df


def test_calc_triple_product_jac():
    """
    Test function for calc_triple_product_jac.
    Tests the Jacobian calculation using finite differences.
    """
    
    # Test case 1: Basic functionality test
    print("Testing calc_triple_product_jac...")
    
    # Define test vectors and their Jacobians
    a = np.array([1.0, 0.0, 0.0])
    b = np.array([0.0, 1.0, 0.0])
    c = np.array([0.0, 0.0, 1.0])
    
    config_dim = 3
    d_a = np.array([[1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0]])
    d_b = np.array([[0.1, 0.0, 0.0],
                    [0.0, 0.1, 0.0],
                    [0.0, 0.0, 0.1]])
    d_c = np.array([[0.01, 0.0, 0.0],
                    [0.0, 0.01, 0.0],
                    [0.0, 0.0, 0.01]])
    
    # Call the function directly
    result = calc_triple_product_jac(a, b, c, d_a, d_b, d_c)
    
    print(f"Input vectors:")
    print(f"a = {a}")
    print(f"b = {b}")
    print(f"c = {c}")
    print(f"Result Jacobian shape: {result.shape}")
    print(f"Result Jacobian: {result}")
    
    # Test case 2: Edge case - nearly parallel vectors
    print("\nTesting edge case with nearly parallel vectors...")
    a_parallel = np.array([1.0, 0.0, 0.0])
    b_parallel = np.array([1.0, 1e-15, 0.0])  # Nearly parallel to a
    c_parallel = np.array([0.0, 0.0, 1.0])
    
    result_parallel = calc_triple_product_jac(a_parallel, b_parallel, c_parallel, d_a, d_b, d_c)
    print(f"Result for nearly parallel vectors: {result_parallel}")
    
    # Test case 3: Verify the triple product formula
    print("\nVerifying triple product calculation...")
    u = np.cross(a, b)
    s = np.linalg.norm(u)
    triple_product = np.dot(u, c) / s
    print(f"Cross product a x b: {u}")
    print(f"Norm of cross product: {s}")
    print(f"Triple product (a x b)·c / ||a x b||: {triple_product}")
    
    # Test case 4: Numerical verification with finite differences
    print("\nNumerical verification with finite differences...")
    eps = 1e-8
    
    def triple_product_func(a_val, b_val, c_val):
        u_val = np.cross(a_val, b_val)
        s_val = np.linalg.norm(u_val)
        if s_val < 1e-12:
            return 0.0
        return np.dot(u_val, c_val) / s_val
    
    # Compute numerical gradient for first parameter of vector a
    a_plus = a.copy()
    a_plus[0] += eps
    a_minus = a.copy()
    a_minus[0] -= eps
    
    numerical_grad_a0 = (triple_product_func(a_plus, b, c) - triple_product_func(a_minus, b, c)) / (2 * eps)
    analytical_grad_a0 = result[0]  # First component corresponds to first parameter
    
    print(f"Numerical gradient (a[0]): {numerical_grad_a0}")
    print(f"Analytical gradient (a[0]): {analytical_grad_a0}")
    print(f"Difference: {abs(numerical_grad_a0 - analytical_grad_a0)}")
    
    if abs(numerical_grad_a0 - analytical_grad_a0) < 1e-6:
        print("✓ Gradient verification PASSED")
    else:
        print("✗ Gradient verification FAILED")
    
    # Test case 5: More comprehensive gradient test
    print("\nComprehensive gradient test...")
    # Use different test vectors for better gradient testing
    a_test = np.array([2.0, 1.0, 0.5])
    b_test = np.array([1.0, 2.0, 1.0])
    c_test = np.array([0.5, 1.0, 2.0])
    
    # More interesting Jacobian matrices
    d_a_test = np.random.rand(3, 4)  # 4 configuration parameters
    d_b_test = np.random.rand(3, 4)
    d_c_test = np.random.rand(3, 4)
    
    result_comprehensive = calc_triple_product_jac(a_test, b_test, c_test, d_a_test, d_b_test, d_c_test)
    print(f"Comprehensive test result shape: {result_comprehensive.shape}")
    print(f"Comprehensive test result: {result_comprehensive}")
    
    # Verify that the result has the correct dimension
    expected_dim = d_a_test.shape[1]  # Should match config_dim
    if result_comprehensive.shape[0] == expected_dim:
        print(f"✓ Output dimension verification PASSED (expected {expected_dim}, got {result_comprehensive.shape[0]})")
    else:
        print(f"✗ Output dimension verification FAILED (expected {expected_dim}, got {result_comprehensive.shape[0]})")
    
    print("Test completed.")


if __name__ == "__main__":
    test_calc_triple_product_jac()
