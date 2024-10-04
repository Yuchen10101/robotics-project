% jacob_T.m
% Descrption: A custom function calculating Jacobian matrix in tool frame
% Input: q (1xN joint configuration); DH (a struct containing d, a, alpha, offset)
% Output: J_T (Jacobian matrix in tool frame)

function J_T = jacob_T(q,DH)

c2 = cos(q(2));     s2 = sin(q(2));     a2 = DH.a(2);       d2 = DH.d(2);
c3 = cos(q(3));     s3 = sin(q(3));
c23 = cos(q(2)+q(3));     s23 = sin(q(2)+q(3));     a3 = DH.a(3);
c4 = cos(q(4));     s4 = sin(q(4));     d4 = DH.d(4);
c234 = cos(q(2)+q(3)+q(4));     s234 = sin(q(2)+q(3)+q(4));
c34 = cos(q(3)+q(4));     s34 = sin(q(3)+q(4));
c5 = cos(q(5));     s5 = sin(q(5));     d5 = DH.d(5);
c6 = cos(q(6));     s6 = sin(q(6));     d6 = DH.d(6);

%% J1_T
J1_T = [c6*s5*(a2*c2 + a3*c23 - d5*s234 + c234*d6*s5) - (s6*s234 - c5*c6*c234)*(d2 + d4 + c5*d6);
      - (c6*s234 + c5*c234*s6)*(d2 + d4 + c5*d6) - s5*s6*(a2*c2 + a3*c23 - d5*s234 + c234*d6*s5);
                          c234*s5*(d2 + d4 + c5*d6) - c5*(a2*c2 + a3*c23 - d5*s234 + c234*d6*s5);
                                                                            c234*s6 + c5*c6*s234;
                                                                            c6*c234 - c5*s6*s234;
                                                                                         s5*s234];

%% J2_T
J2_T = [(c34*s6 + c5*c6*s34)*(a2 + a3*c3 - d5*s34 + c34*d6*s5) + (s6*s34 - c5*c6*c34)*(c34*d5 + a3*s3 + d5*s5*s34);
        (c6*c34 - c5*s6*s34)*(a2 + a3*c3 - d5*s34 + c34*d6*s5) + (c6*s34 + c5*c34*s6)*(c34*d5 + a3*s3 + d5*s5*s34);
                                     s4*s5*(a2 + a3*c3 - d5*s34 + c34*d6*s5) - c34*s5*(c34*d5 + a3*s3 + d5*s5*s34);
                                                                                                            -c6*s5;
                                                                                                             s5*s6;
                                                                                                                c5];

%% J3_T
J3_T = [(c4*s6 + c5*c6*s4)*(a3 - d5*s4 + c4*d6*s5) + (s4*s6 - c4*c5*c6)*(c4*d5 + d6*s4*s5);
        (c4*c6 - c5*s4*s6)*(a3 - d5*s4 + c4*d6*s5) + (c6*s4 + c4*c5*s6)*(c4*d5 + d6*s4*s5);
                                  s4*s5*(a3 - d5*s4 + c4*d6*s5) - c4*s5*(c4*d5 + d6*s4*s5);
                                                                                    -c6*s5;
                                                                                     s5*s6;
                                                                                        c5];

%% J4_T
J4_T = [d6*s5*s6 - c5*c6*d5;
        c5*d5*s6 + c6*d6*s5;
                     -d5*s5;
                     -c6*s5;
                      s5*s6;
                         c5];

%% J5_T
J5_T = [c6*d6;
       -d6*s6;
            0;
           s6;
           c6;
            0];

%% J6_T

J6_T = [0;
        0;
        0;
        0;
        0;
        1];

%% J_T

J_T = [J1_T J2_T J3_T J4_T J5_T J6_T];

end
