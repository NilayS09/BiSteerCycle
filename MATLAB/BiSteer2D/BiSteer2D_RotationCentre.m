function rR_C = BiSteer2D_RotationCentre(delF,delR,lF,lR,psi)
%BiSteer2D_RotationCentre
%    rR_C = BiSteer2D_RotationCentre(delF,delR,lF,lR,PSI)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    05-Jun-2023 07:09:08

t2 = cos(delF);
t3 = cos(delR);
t4 = cos(psi);
t5 = sin(delF);
t6 = sin(delR);
t7 = sin(psi);
t8 = lF+lR;
t9 = t2.*t6;
t10 = t3.*t5;
t11 = -t10;
t12 = t9+t11;
t13 = 1.0./t12;
rR_C = [-t2.*t8.*t13.*(t3.*t7+t4.*t6);t2.*t8.*t13.*(t3.*t4-t6.*t7);0.0];
end
