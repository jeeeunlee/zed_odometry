function SE3 = InverseSE3(matrix)

R=matrix(1:3,1:3);
P=matrix(1:3,4);

SE3=[transpose(R) -transpose(R)*P;
    0 0 0 1];
