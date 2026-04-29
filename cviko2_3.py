import cvxpy as cp

x = cp.Variable(8)
c = [10, 8, 6, 2, 4, 5, 8, 9]
objective = cp.Minimize(x@c)

constraints = [
    0 <= x,
    x[0] == x[2] + x[3],
    x[1] == x[4] + x[5],
    x[2] + x[4] == x[6],
    x[3] + x[5] == x[7],
]

problem = cp.Problem(objective, constraints)
problem.solve()

print(objective.value)
print(x.value)

