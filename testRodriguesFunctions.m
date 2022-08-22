% test/fix rodrigues rotation paramterization functions

r = [-1 -1 -2.8]
R = rod2mat(r(1), r(2), r(3))

mat2rod(R)
norm(r)
