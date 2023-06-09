function D = functionD(Ixx2,Ixx3,Ixy2,Ixy3,Ixz2,Ixz3,Iyy2,Iyy3,Iyz2,Iyz3,Izz1,Izz2,Izz3,L2,Lcom2,Lcom3,m2,m3,q1,q2,q3)
%functionD
%    D = functionD(Ixx2,Ixx3,Ixy2,Ixy3,Ixz2,Ixz3,Iyy2,Iyy3,Iyz2,Iyz3,Izz1,Izz2,Izz3,L2,Lcom2,Lcom3,M2,M3,Q1,Q2,Q3)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    16-Apr-2023 13:16:18

t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = sin(q1);
t6 = sin(q2);
t7 = q1.*2.0;
t8 = q1.*4.0;
t9 = q2.*2.0;
t10 = q3.*2.0;
t11 = q1.*3.0;
t12 = q2+q3;
t13 = L2.^2;
t14 = Lcom2.^2;
t15 = Lcom3.^2;
t16 = Ixx3.*2.5e-1;
t17 = Iyy3.*2.5e-1;
t18 = Izz3.*5.0e-1;
t19 = cos(t7);
t20 = cos(t9);
t21 = Ixy2.*t2;
t22 = Ixy3.*t2;
t23 = Iyy2.*t2;
t24 = Iyy3.*t2;
t25 = Iyz2.*t2;
t26 = Iyz3.*t2;
t27 = sin(t7);
t28 = Ixx2.*t5;
t29 = Ixx3.*t5;
t30 = Ixy2.*t5;
t31 = Ixy3.*t5;
t32 = cos(t12);
t33 = sin(t12);
t34 = m3.*t15;
t35 = Ixz2.*t5.*-1.0;
t36 = Ixz2.*t5.*1.0;
t37 = Ixz3.*t5.*-1.0;
t38 = Ixz3.*t5.*1.0;
t39 = q1+t9;
t40 = L2.*Lcom3.*m3.*t4;
t50 = t7+t9;
t51 = t8+t9;
t52 = t9+t10;
t53 = t9+t11;
t41 = Ixz3.*t32;
t42 = Iyz3.*t32;
t43 = Ixz3.*t33;
t44 = Iyz3.*t33;
t45 = t16.*t19;
t46 = Iyy3.*t19.*-2.5e-1;
t47 = t17.*t19;
t48 = Ixy3.*t27.*-5.0e-1;
t49 = Ixy3.*t27.*5.0e-1;
t54 = t22.*t32;
t55 = Ixx3.*t2.*t33;
t56 = t31.*t32;
t57 = Iyy3.*t5.*t33;
t58 = cos(t50);
t59 = cos(t51);
t60 = cos(t52);
t61 = t10+t39;
t62 = sin(t52);
t64 = Ixx3.*t2.*t32.*-1.0;
t65 = Ixx3.*t2.*t32.*1.0;
t67 = t21+t28;
t68 = t22+t29;
t69 = t23+t30;
t70 = t24+t31;
t71 = t22.*t33.*-1.0;
t72 = t22.*t33.*1.0;
t73 = Iyy3.*t5.*t32.*-1.0;
t74 = Iyy3.*t5.*t32.*1.0;
t75 = t31.*t33.*-1.0;
t76 = t31.*t33.*1.0;
t77 = t10+t50;
t78 = t10+t51;
t79 = t25+t35;
t80 = t26+t37;
t81 = t11+t52;
t63 = cos(t61);
t66 = sin(t61);
t82 = sin(t78);
t83 = cos(t81);
t84 = sin(t81);
t85 = Ixx3.*t60.*-1.25e-1;
t86 = Ixx3.*t60.*1.25e-1;
t87 = Iyy3.*t60.*1.25e-1;
t88 = Ixy3.*t62.*-2.5e-1;
t89 = Ixy3.*t62.*2.5e-1;
t90 = cos(t77);
t91 = cos(t78);
t106 = t32.*t80;
t107 = t33.*t80;
t108 = t2.*t32.*t68;
t109 = t5.*t33.*t70;
t110 = t2.*t33.*t68.*-1.0;
t111 = t2.*t33.*t68.*1.0;
t112 = t5.*t32.*t70.*-1.0;
t113 = t5.*t32.*t70.*1.0;
t114 = t43+t56+t64;
t115 = t44+t54+t73;
t116 = t41+t55+t75;
t117 = t42+t57+t71;
t92 = Iyz3.*t63.*5.0e-1;
t93 = Ixz3.*t66.*5.0e-1;
t94 = Ixz3.*t84.*5.0e-1;
t95 = Ixx3.*t90.*-2.5e-1;
t96 = t16.*t90;
t97 = Ixx3.*t91.*-1.25e-1;
t98 = Ixx3.*t91.*1.25e-1;
t99 = Iyy3.*t90.*-2.5e-1;
t100 = t17.*t90;
t101 = Iyy3.*t91.*1.25e-1;
t102 = t18.*t90;
t103 = Ixy3.*t82.*2.5e-1;
t104 = Iyz3.*t83.*-5.0e-1;
t105 = Iyz3.*t83.*5.0e-1;
t118 = t2.*t115;
t119 = t2.*t117;
t120 = t5.*t114.*-1.0;
t121 = t5.*t114.*1.0;
t122 = t5.*t116.*-1.0;
t123 = t5.*t116.*1.0;
t124 = t106+t109+t110;
t125 = t107+t108+t112;
t126 = t5.*t125;
t127 = t2.*t124.*-1.0;
t128 = t2.*t124.*1.0;
t129 = t118+t120;
t130 = t119+t122;
t134 = t16+t17+t18+t34+t40+t45+t46+t48+t85+t87+t88+t92+t93+t94+t95+t97+t99+t101+t102+t103+t104;
t131 = t5.*t129;
t132 = t2.*t130.*-1.0;
t133 = t2.*t130.*1.0;
mt1 = [Ixx2.*5.0e-1+Ixx3.*5.0e-1+Iyy2.*5.0e-1+Iyy3.*5.0e-1+Izz1+t34.*5.0e-1+t40-Ixx2.*t19.*5.0e-1-Ixx3.*t19.*5.0e-1+Ixy2.*t27+Ixy3.*t27+Iyy2.*t19.*5.0e-1+Iyy3.*t19.*5.0e-1+m2.*t14.*5.0e-1+m3.*t13.*5.0e-1+t34.*t60.*5.0e-1+m2.*t14.*t20.*5.0e-1+m3.*t13.*t20.*5.0e-1+L2.*Lcom3.*m3.*cos(q3+t9),t131+t132-t5.*(t5.*(Ixz2.*t6+t3.*t30-Ixx2.*t2.*t3.*1.0).*1.0-t2.*(Iyz2.*t6+t3.*t21-Iyy2.*t3.*t5.*1.0))+t2.*(t5.*(Ixz2.*t3-t6.*t30.*1.0+Ixx2.*t2.*t6).*1.0-t2.*(Iyz2.*t3-t6.*t21.*1.0+Iyy2.*t5.*t6)).*1.0,t131+t132,t126+t127-t2.*(t3.*t79-t2.*t6.*t67.*1.0+t5.*t6.*t69).*1.0+t5.*(t6.*t79+t2.*t3.*t67-t3.*t5.*t69.*1.0)];
mt2 = [Ixx2.*2.5e-1+Iyy2.*2.5e-1+Izz2.*5.0e-1+t16+t17+t18+t34+t40.*2.0+t45+t46+t48+t85+t87+t88+t92+t93+t94+t95+t97+t99+t101+t102+t103+t104+Ixx2.*t19.*2.5e-1-Ixx2.*t20.*1.25e-1-Ixx2.*t58.*2.5e-1-Ixx2.*t59.*1.25e-1-Ixy2.*t27.*5.0e-1-Iyy2.*t19.*2.5e-1+Iyy2.*t20.*1.25e-1-Iyy2.*t58.*2.5e-1+Iyy2.*t59.*1.25e-1+Izz2.*t58.*5.0e-1+m2.*t14+m3.*t13+Iyz2.*cos(t39).*5.0e-1-Iyz2.*cos(t53).*5.0e-1-Ixy2.*sin(t9).*2.5e-1+Ixy2.*sin(t51).*2.5e-1+Ixz2.*sin(t39).*5.0e-1+Ixz2.*sin(t53).*5.0e-1,t134,t126+t127,t134,t16+t17+t18+t34+t45+t46+t48+t85+t87+t88+t92+t93+t94+t95+t97+t99+t101+t102+t103+t104];
D = reshape([mt1,mt2],3,3);
