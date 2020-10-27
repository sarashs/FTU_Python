#generate random binary string of length n

import random

#create empty string using a key
def rand_key(p):
    key = ""

    for i in range(p):
        temp = str(random.randint(0,1)) #generate 1 or 0
        key += temp #concat string

    return key

#main
n = 16
str1 = rand_key(n)
print(str1)

