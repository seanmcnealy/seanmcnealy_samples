from itertools import product, chain
from math import log2, log10, ceil

# Problem can be found at:
# https://docs.google.com/document/d/16v0lgBsyT9MsKtm9JHdAyRl8YpIy5Zk5r7haXLC_JBU/edit?usp=sharing

def solve(key_string, find):
    buttons = list(filter(lambda x: x >= 0, map(lambda x: -1 if x[0] == "0" else x[1], zip(key_string, range(10)) )))
    numbers = []
    for repeat in range(1, ceil(log10(find)) + 1):
        numbers.append(list(product(buttons, repeat=repeat)))
    base_case = list(map(lambda x: int("".join(map(str,x))), list(chain(*numbers))))

    X = {}
    path = {}
    for x in base_case:
        if x <= find:
            X[x] = len(str(x))
            path[x] = str(x)

    for i in range(ceil(log2(find)) + 1):
        if find in X:
            break
        for x, y in product(X.keys(), repeat=2):
            p = x * y
            if p <= find and (p not in X or X[p] > X[x] + X[y] + 1):
                X[p] = X[x] + X[y] + 1
                path[p] = path[x] + "*" + path[y]
    if find in X:
        return X[find] + 1, path[find] + "="
    else:
        return "Impossible", None


test1 = "0 1 1 0 0 1 0 0 0 0".split(" ")
find = 60
answer, solution = solve(test1, find)
print("Case #1:", answer, solution)

test2 = "1 1 1 1 1 1 1 1 1 1".split(" ")
find2 = 128
answer, solution = solve(test2, find2)
print("Case #2:", answer, solution)

test3 = "0 1 0 1 0 1 0 1 0 1".split(" ")
find3 = 128
answer, solution = solve(test3, find3)
print("Case #3:", answer, solution)
