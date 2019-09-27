function point=extendPoint(q_near, q_rand, stepsize)
p=q_rand-q_near;
v=p./norm(p);
point=double(int32(q_near+v*stepsize));
end