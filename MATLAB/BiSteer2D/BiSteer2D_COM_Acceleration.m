function Ac = BiSteer2D_COM_Acceleration(Vr,Vr_dot,delF,delR,delF_dot,delR_dot,lF,lR,psi)
%BiSteer2D_COM_Acceleration
%    Ac = BiSteer2D_COM_Acceleration(Vr,Vr_dot,delF,delR,delF_dot,delR_dot,lF,lR,PSI)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    05-Jun-2023 12:30:35

t2 = cos(delF);
t3 = cos(delR);
t4 = cos(psi);
t5 = sin(delR);
t6 = sin(psi);
t7 = lF+lR;
t8 = Vr.^2;
t9 = delF.*2.0;
t11 = -delR;
t10 = Vr_dot.*t5;
t12 = t3.*t6;
t13 = t4.*t5;
t14 = t5.*t6;
t15 = 1.0./t2;
t17 = Vr.*delR_dot.*t3;
t18 = t3.*t4;
t19 = delF+t11;
t20 = 1.0./t7;
t22 = Vr.*delF_dot.*t3.*2.0;
t26 = t9+t11;
t16 = t15.^2;
t21 = t20.^2;
t23 = -t22;
t24 = sin(t19);
t25 = -t18;
t27 = cos(t26);
t28 = sin(t26);
t31 = t12+t13;
t29 = t24.^2;
t30 = Vr_dot.*t28;
t32 = Vr.*delR_dot.*t27;
t34 = t14+t25;
t35 = Vr.*t15.*t20.*t24;
t33 = -t30;
t36 = delR_dot+t35;
t37 = t10+t17+t23+t32+t33;
Ac = -t4.*(-Vr_dot.*t31+Vr.*t34.*t36+(lR.*t4.*t16.*t20.*t37)./2.0+lR.*t6.*t8.*t16.*t21.*t29)+t6.*(Vr_dot.*t34+Vr.*t31.*t36-(lR.*t6.*t16.*t20.*t37)./2.0+lR.*t4.*t8.*t16.*t21.*t29);
end
