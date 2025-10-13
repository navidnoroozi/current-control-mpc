import math

def clarke_trans_calc_in_balanced(abc: list) -> list:
    """Clarke transformation for balanced three-phase systems.

    Args:
        abc (list): A list of three elements representing the three-phase
                    quantities [a, b, c].
    Returns:
        list: A list of two elements representing the alpha-beta components 
        [alpha, beta].
    """
    if len(abc) != 3:
        raise ValueError("Input list must have exactly three elements " \
                        "representing phases a, b, and c.")
    
    a, b, c = abc
    alpha = (2/3) * (a - 0.5 * (b + c))
    beta = (2/3) * ((math.sqrt(3)/2) * (b - c))
    
    return [alpha, beta]