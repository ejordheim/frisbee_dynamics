def intrpf(xi,x,y):
    """
    Function to interpolate between data points
    using Lagrange polynomial (quadratic)
    Inputs
       x    Vector of x coordinates of data points (n values)
       y    Vector of y coordinates of data points (n values)
       xi   The x value where interpolation is computed
    Output
      yi   The interpolation polynomial evaluated at xi
    """

    # Calculate yi = p(xi) using Lagrange polynomial
    yi = 0
    for i in range(0,len(x)):
        prod = 1
        for j in range(0,len(x)):
            if i != j:
                prod = prod * ((xi - x[j]) / (x[i] - x[j]))
        yi = yi + (prod * y[i])
    return yi;