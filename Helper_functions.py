# -*- coding: utf-8 -*-
"""
Created on Wed May 29 00:30:57 2024

@author: 88chr
"""
import numpy as np

def flattenNestedListToTuple(nestedListOfNumbers):
    array = np.array(nestedListOfNumbers)

    # Flatten the array
    flattened_array = array.flatten()

    # Convert the flattened array to a tuple
    flattened_tuple = tuple(flattened_array)
    return flattened_tuple


