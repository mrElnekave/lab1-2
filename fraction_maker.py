

def fraction_maker(number: float):
    """Takes in a float and returns tuple of fraction"""
    delta = 0.0000001
    if number == 0:
        return 0, 0
    elif number == 1:
        return 1, 1
    else:
        numerator = 1
        denominator = 1
        while numerator / denominator - number > delta or numerator / denominator - number < -delta:
            if numerator / denominator > number:
                denominator += 1
            else:
                numerator += 1
        return numerator, denominator


""" This is a solver for my curcuits class """


def delta_wye_transform(r_a, r_b, r_c):
    """
    Transforms a delta network to a wye network

    Args:
        r_a: resistance of branch a
        r_b: resistance of branch b
        r_c: resistance of branch c
    """

    sum = r_a + r_b + r_c
    r_1 = r_b * r_c / sum
    r_2 = r_a * r_c / sum
    r_3 = r_a * r_b / sum

    return r_1, r_2, r_3


def wye_delta_transfrom(r_1, r_2, r_3):
    """
    Transforms a wye network to a delta network

    Args:
        r_1: resistance of branch 1
        r_2: resistance of branch 2
        r_3: resistance of branch 3
    """

    sum_of_prods = r_1 * r_2 + r_2 * r_3 + r_1 * r_3
    r_a = sum_of_prods / r_1
    r_b = sum_of_prods / r_2
    r_c = sum_of_prods / r_3

    return r_a, r_b, r_c


def parallel_res(*args):
    """
    Calculates the parallel resistance of a list of resistors

    Args:
        args: list of resistors
    """
    sum = 0
    for r in args:
        sum += 1/r
    return 1/sum
