function [x_dot,y_dot,psi_dot,Vr_dot] = BiSteer2D_PoseDerivatives(I33,Tf,Tr,Vr,delF,delR,delF_dot,delR_dot,lF,lR,m,psi)
%BiSteer2D_PoseDerivatives
%    [X_DOT,Y_DOT,PSI_DOT,Vr_dot] = BiSteer2D_PoseDerivatives(I33,Tf,Tr,Vr,delF,delR,delF_dot,delR_dot,lF,lR,M,PSI)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    05-Jun-2023 07:09:08

t2 = cos(delF);
t3 = cos(delR);
t4 = sin(delF);
t5 = sin(delR);
t6 = delR+psi;
x_dot = Vr.*cos(t6);
if nargout > 1
    y_dot = Vr.*sin(t6);
end
if nargout > 2
    t7 = delF.*2.0;
    t8 = delR.*2.0;
    t9 = lF.^2;
    t10 = lR.^2;
    t11 = cos(t7);
    t12 = cos(t8);
    t13 = t2.^2;
    t14 = t2.^3;
    t15 = t3.^2;
    t16 = -t8;
    t17 = 1.0./t2;
    psi_dot = (Vr.*t17.*sin(delF-delR))./(lF+lR);
end
if nargout > 3
    t18 = t7+t16;
    t19 = cos(t18);
    Vr_dot = (t17.*(Tr.*t9.*t14+Tr.*t10.*t14+Tr.*lF.*lR.*t14.*2.0+Tf.*t3.*t9.*t13+Tf.*t3.*t10.*t13-I33.*Vr.*delF_dot.*t4.*t15-I33.*Vr.*delR_dot.*t4.*t13+Tf.*lF.*lR.*t3.*t13.*2.0+I33.*Vr.*delF_dot.*t2.*t3.*t5+I33.*Vr.*delR_dot.*t2.*t3.*t5-I33.*Vr.*delR_dot.*t3.*t5.*t14.*2.0+I33.*Vr.*delR_dot.*t4.*t13.*t15.*2.0-Vr.*delF_dot.*m.*t4.*t10.*t15+Vr.*delR_dot.*lF.*lR.*m.*t4.*t13+Vr.*delR_dot.*m.*t2.*t3.*t5.*t10-Vr.*delF_dot.*lF.*lR.*m.*t2.*t3.*t5+Vr.*delR_dot.*lF.*lR.*m.*t3.*t5.*t14.*2.0-Vr.*delR_dot.*lF.*lR.*m.*t4.*t13.*t15.*2.0).*2.0)./(I33-I33.*t19+m.*t9+m.*t10+lF.*lR.*m+m.*t9.*t11+m.*t10.*t12+lF.*lR.*m.*t11+lF.*lR.*m.*t12+lF.*lR.*m.*t19);
end
end
