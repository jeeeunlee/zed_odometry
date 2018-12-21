function R = EulerZYX(q_z, q_y, q_x)

cz = cos(q_z);
sz = sin(q_z);
cy = cos(q_y);
sy = sin(q_y);
cx = cos(q_x);
sx = sin(q_x);

R = [ cz * cy  cz * sy * sx - sz * cx  cz * sy * cx + sz * sx;
      sz * cy  sz * sy * sx + cz * cx  sz * sy * cx - cz * sx;
      - sy     cy * sx                 cy * cx                ];
