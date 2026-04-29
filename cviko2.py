import cvxpy as cp

x = cp.Variable(2, integer=True)
objective = cp.Maximize(0.25*x[0]+0.075*x[1])
constraints = [
    x >= 0,
    x[0] + x[1] <= 60,
    2.5*x[0]+0.5*x[1] <= 50,
]

problem = cp.Problem(objective, constraints)
problem.solve()

print(objective.value)
print(x.value)
