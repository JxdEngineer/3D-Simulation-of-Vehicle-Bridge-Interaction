function [a0,a1]=DampRayleigh(omega1,omega2,zeta1,zeta2)
%定义Rayleigh阻尼
%omega是自振圆频率，zeta是阻尼比
a=2*omega1*omega2/(omega2^2-omega1^2) ...
    *[omega2,-omega1;-1/omega2,1/omega1] ...
    *[zeta1;zeta2];
a0=a(1);
a1=a(2);
end