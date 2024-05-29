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


# def checkTupleAndSetDefault(thisTuple, defaultTuple, verbose = False):
#     if thisTuple[0] is None:
#         return defaultTuple
#     for value in 


#     if (thisTuple is None or not isinstance(thisTuple, tuple)):
#         if verbose:
#             print("it's a none")
#         return defaultTuple
#     return thisTuple


def tryToChangeTuple(thisTuple):
    thisTuple = (1, 3, 5)
    return thisTuple

A = (1, 2)
B = tryToChangeTuple(A)
print(B)
print(A)
TupNones = (None, None)
if TupNones[0] is None:
    print("thudaslfslaj")
    
C = tuple(zip(A, TupNones))
print(type(C))
print(str(C))


AList = [1, 2, 3]
AList.append(4)
print(AList)
A_np = np.array(AList)
B_np = np.array(A)
print(A_np)
print(B_np)
Nones_np = np.array(TupNones)
print(Nones_np)
if Nones_np[1] == None :
    print("equal none")