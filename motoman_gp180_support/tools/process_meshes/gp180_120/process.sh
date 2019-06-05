#/bin/bash

stl_transform -rx 90 -s 0.001 input/BASE_AXIS.stl output/base_link.stl
stl_transform -rx 90 -s 0.001 -tz -0.317 input/S_AXIS.stl output/link_1_s.stl
stl_transform -rx 90 -s 0.001 -tx 0.325 -ty 0.07 -tz -0.65 input/L_AXIS.stl output/link_2_l.stl
stl_transform -rx 90 -s 0.001 -tx 0.325 -ty 0.0415 -tz -1.8 input/U_AXIS.stl output/link_3_u.stl
stl_transform -rx 90 -s 0.001 -tx 1.723 -tz -2.1 input/R_AXIS.stl output/link_4_r.stl
stl_transform -rx 90 -s 0.001 -tx 1.915 -ty -0.082 -tz -2.1 input/B_AXIS.stl output/link_5_b.stl
stl_transform -rx 90 -s 0.001 -tx 2.1145 -tz -2.1 input/T_AXIS.stl output/link_6_t.stl