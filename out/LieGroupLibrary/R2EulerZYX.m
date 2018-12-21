%%  -90 < q_y < 90
function angles = R2EulerZYX(R)

q_y=atan2(-R(3,1),sqrt( R(1,1)^2+R(2,1)^2));
q_z=atan2(R(2,1),R(1,1));
q_x=atan2(R(3,2),R(3,3));

 
angles=[ q_x ; q_y ; q_z ];
