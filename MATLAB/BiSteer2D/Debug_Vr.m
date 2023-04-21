function [num,den,Vr_dot] = Debug_Vr(I33,Tf,Tr,Vr,delF,delR,delF_dot,delR_dot,lF,lR,m)
%Debug_Vr
%    [NUM,DEN,Vr_dot] = Debug_Vr(I33,Tf,Tr,Vr,delF,delR,delF_dot,delR_dot,lF,lR,M)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    07-Apr-2023 18:59:25

t2 = cos(delF);
t3 = cos(delR);
t4 = sin(delF);
t5 = sin(delR);
t6 = delF.*2.0;
t7 = delR.*2.0;
t8 = lF.^2;
t9 = lR.^2;
t15 = lF.*lR.*m;
t10 = cos(t6);
t11 = cos(t7);
t12 = t2.^2;
t13 = t2.^3;
t14 = t3.^2;
t16 = -t7;
t17 = m.*t8;
t18 = m.*t9;
num = Tr.*t8.*t13.*2.0+Tr.*t9.*t13.*2.0+Tr.*lF.*lR.*t13.*4.0+Tf.*t3.*t8.*t12.*2.0+Tf.*t3.*t9.*t12.*2.0-I33.*Vr.*delF_dot.*t4.*t14.*2.0-I33.*Vr.*delR_dot.*t4.*t12.*2.0+Tf.*lF.*lR.*t3.*t12.*4.0-Vr.*delF_dot.*t4.*t14.*t18.*2.0+Vr.*delR_dot.*t4.*t12.*t15.*2.0+I33.*Vr.*delF_dot.*t2.*t3.*t5.*2.0+I33.*Vr.*delR_dot.*t2.*t3.*t5.*2.0-I33.*Vr.*delR_dot.*t3.*t5.*t13.*4.0+I33.*Vr.*delR_dot.*t4.*t12.*t14.*4.0-Vr.*delF_dot.*t2.*t3.*t5.*t15.*2.0+Vr.*delR_dot.*t2.*t3.*t5.*t18.*2.0+Vr.*delR_dot.*t3.*t5.*t13.*t15.*4.0-Vr.*delR_dot.*t4.*t12.*t14.*t15.*4.0;
if nargout > 1
    t19 = t10.*t15;
    t20 = t11.*t15;
    t21 = t6+t16;
    t23 = t10.*t17;
    t24 = t11.*t18;
    t22 = cos(t21);
    t25 = I33.*t22;
    t26 = t15.*t22;
    t27 = -t25;
    t28 = I33+t15+t17+t18+t19+t20+t23+t24+t26+t27;
    den = t2.*t28;
end
if nargout > 2
    Vr_dot = ((Tr.*t8.*t13+Tr.*t9.*t13+Tr.*lF.*lR.*t13.*2.0+Tf.*t3.*t8.*t12+Tf.*t3.*t9.*t12-I33.*Vr.*delF_dot.*t4.*t14-I33.*Vr.*delR_dot.*t4.*t12+Tf.*lF.*lR.*t3.*t12.*2.0-Vr.*delF_dot.*t4.*t14.*t18+Vr.*delR_dot.*t4.*t12.*t15+I33.*Vr.*delF_dot.*t2.*t3.*t5+I33.*Vr.*delR_dot.*t2.*t3.*t5-I33.*Vr.*delR_dot.*t3.*t5.*t13.*2.0+I33.*Vr.*delR_dot.*t4.*t12.*t14.*2.0-Vr.*delF_dot.*t2.*t3.*t5.*t15+Vr.*delR_dot.*t2.*t3.*t5.*t18+Vr.*delR_dot.*t3.*t5.*t13.*t15.*2.0-Vr.*delR_dot.*t4.*t12.*t14.*t15.*2.0).*2.0)./(t2.*t28);
end
end
