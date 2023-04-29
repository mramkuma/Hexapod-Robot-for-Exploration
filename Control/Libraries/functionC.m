function C = functionC(Ixx2,Ixx3,Ixy2,Ixy3,Ixz2,Ixz3,Iyy2,Iyy3,Iyz2,Iyz3,Izz2,Izz3,L2,Lcom2,Lcom3,dq1,dq2,dq3,m2,m3,q1,q2,q3)
%functionC
%    C = functionC(Ixx2,Ixx3,Ixy2,Ixy3,Ixz2,Ixz3,Iyy2,Iyy3,Iyz2,Iyz3,Izz2,Izz3,L2,Lcom2,Lcom3,DQ1,DQ2,DQ3,M2,M3,Q1,Q2,Q3)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    16-Apr-2023 13:16:20

t2 = cos(q1);
t3 = cos(q2);
t4 = sin(q1);
t5 = sin(q2);
t6 = sin(q3);
t7 = q1.*-1.0;
t8 = q1.*1.0;
t9 = q1.*2.0;
t10 = q2.*-1.0;
t11 = q2.*1.0;
t12 = q1.*4.0;
t13 = q2.*2.0;
t14 = q3.*2.0;
t15 = q1.*3.0;
t16 = q2+q3;
t17 = L2.^2;
t18 = Lcom2.^2;
t19 = Lcom3.^2;
t20 = cos(t9);
t21 = sin(t9);
t22 = sin(t13);
t23 = cos(t16);
t24 = sin(t16);
t25 = q2+t15;
t26 = q1+t10;
t27 = q1+t13;
t28 = q2+t9;
t29 = q3+t13;
t35 = t7+t16;
t36 = t9+t16;
t44 = t15+t16;
t46 = t9+t13;
t47 = t12+t13;
t48 = t13+t14;
t49 = L2.*Lcom3.*m3.*t6.*5.0e-1;
t50 = t13+t15;
t30 = cos(t26);
t31 = cos(t27);
t32 = cos(t28);
t33 = Ixz3.*t23;
t34 = Iyz3.*t23;
t37 = sin(t26);
t38 = sin(t27);
t39 = sin(t28);
t40 = sin(t29);
t41 = Ixz3.*t24;
t42 = Iyz3.*t24;
t43 = cos(t25);
t45 = sin(t25);
t51 = cos(t35);
t52 = cos(t36);
t53 = sin(t35);
t54 = sin(t36);
t55 = cos(t44);
t56 = sin(t44);
t63 = cos(t50);
t64 = sin(t50);
t65 = Ixy3.*t2.*t23;
t66 = Ixx3.*t2.*t24;
t67 = Ixy3.*t4.*t23;
t68 = Iyy3.*t4.*t24;
t69 = Ixy3.*dq2.*t20.*5.0e-1;
t70 = Ixy3.*dq2.*t20.*-1.0;
t71 = Ixy3.*dq2.*t20.*1.0;
t72 = Ixy3.*dq3.*t20.*5.0e-1;
t73 = Ixy3.*dq3.*t20.*-1.0;
t74 = Ixy3.*dq3.*t20.*1.0;
t75 = Ixx3.*dq2.*t21.*2.5e-1;
t76 = Ixx3.*dq2.*t21.*-5.0e-1;
t77 = Ixx3.*dq2.*t21.*5.0e-1;
t78 = Ixx3.*dq3.*t21.*2.5e-1;
t79 = Ixx3.*dq3.*t21.*-5.0e-1;
t80 = Ixx3.*dq3.*t21.*5.0e-1;
t81 = Iyy3.*dq2.*t21.*-2.5e-1;
t82 = Iyy3.*dq2.*t21.*2.5e-1;
t83 = Iyy3.*dq2.*t21.*5.0e-1;
t84 = Iyy3.*dq3.*t21.*-2.5e-1;
t85 = Iyy3.*dq3.*t21.*2.5e-1;
t86 = Iyy3.*dq3.*t21.*5.0e-1;
t87 = cos(t47);
t88 = cos(t48);
t89 = t14+t27;
t96 = sin(t46);
t97 = sin(t47);
t98 = sin(t48);
t100 = Ixx3.*t2.*t23.*-1.0;
t101 = Ixx3.*t2.*t23.*1.0;
t103 = Ixy3.*t2.*t24.*-1.0;
t104 = Ixy3.*t2.*t24.*1.0;
t105 = Iyy3.*t4.*t23.*-1.0;
t106 = Iyy3.*t4.*t23.*1.0;
t107 = Ixy3.*t4.*t24.*-1.0;
t108 = Ixy3.*t4.*t24.*1.0;
t109 = t14+t46;
t110 = t14+t47;
t125 = t15+t48;
t57 = dq2.*t42.*-2.5e-1;
t58 = dq2.*t42.*2.5e-1;
t59 = dq2.*t42.*5.0e-1;
t60 = dq3.*t42.*-2.5e-1;
t61 = dq3.*t42.*2.5e-1;
t62 = dq3.*t42.*5.0e-1;
t90 = dq2.*t33.*2.5e-1;
t91 = dq2.*t33.*-5.0e-1;
t92 = dq2.*t33.*5.0e-1;
t93 = dq3.*t33.*2.5e-1;
t94 = dq3.*t33.*-5.0e-1;
t95 = dq3.*t33.*5.0e-1;
t99 = cos(t89);
t102 = sin(t89);
t111 = Ixy2.*dq1.*t30.*2.5e-1;
t112 = Ixy2.*dq2.*t30.*-2.5e-1;
t113 = Ixy2.*dq2.*t30.*2.5e-1;
t114 = Ixz2.*dq2.*t31.*5.0e-1;
t115 = Ixz2.*dq2.*t32.*5.0e-1;
t116 = Ixx2.*dq1.*t37.*1.25e-1;
t117 = Ixx2.*dq2.*t37.*-1.25e-1;
t118 = Ixx2.*dq2.*t37.*1.25e-1;
t119 = Iyy2.*dq1.*t37.*-1.25e-1;
t120 = Iyy2.*dq1.*t37.*1.25e-1;
t121 = Iyy2.*dq2.*t37.*1.25e-1;
t122 = Iyz2.*dq2.*t38.*-5.0e-1;
t123 = Iyz2.*dq2.*t38.*5.0e-1;
t124 = Iyz2.*dq2.*t39.*5.0e-1;
t126 = Ixy3.*dq1.*t51.*2.5e-1;
t127 = Ixy3.*dq1.*t51.*-5.0e-1;
t128 = Ixy3.*dq1.*t51.*5.0e-1;
t129 = Ixy3.*dq2.*t51.*-2.5e-1;
t130 = Ixy3.*dq2.*t51.*2.5e-1;
t131 = Ixy3.*dq2.*t51.*5.0e-1;
t132 = Ixy3.*dq3.*t51.*-2.5e-1;
t133 = Ixy3.*dq3.*t51.*2.5e-1;
t134 = Ixy3.*dq3.*t51.*5.0e-1;
t135 = Ixz3.*dq1.*t52.*-5.0e-1;
t136 = Ixz3.*dq1.*t52.*5.0e-1;
t137 = Ixz3.*dq2.*t52.*-2.5e-1;
t138 = Ixz3.*dq2.*t52.*2.5e-1;
t139 = Ixz3.*dq2.*t52.*5.0e-1;
t140 = Ixz3.*dq3.*t52.*-2.5e-1;
t141 = Ixz3.*dq3.*t52.*2.5e-1;
t142 = Ixz3.*dq3.*t52.*5.0e-1;
t143 = sin(t109);
t144 = sin(t110);
t145 = Ixx3.*dq1.*t53.*-1.25e-1;
t146 = Ixx3.*dq1.*t53.*1.25e-1;
t147 = Ixx3.*dq1.*t53.*2.5e-1;
t148 = Ixx3.*dq2.*t53.*1.25e-1;
t149 = Ixx3.*dq2.*t53.*-2.5e-1;
t150 = Ixx3.*dq2.*t53.*2.5e-1;
t151 = Ixx3.*dq3.*t53.*1.25e-1;
t152 = Ixx3.*dq3.*t53.*-2.5e-1;
t153 = Ixx3.*dq3.*t53.*2.5e-1;
t154 = Iyy3.*dq1.*t53.*1.25e-1;
t155 = Iyy3.*dq1.*t53.*-2.5e-1;
t156 = Iyy3.*dq1.*t53.*2.5e-1;
t157 = Iyy3.*dq2.*t53.*-1.25e-1;
t158 = Iyy3.*dq2.*t53.*1.25e-1;
t159 = Iyy3.*dq2.*t53.*2.5e-1;
t160 = Iyy3.*dq3.*t53.*-1.25e-1;
t161 = Iyy3.*dq3.*t53.*1.25e-1;
t162 = Iyy3.*dq3.*t53.*2.5e-1;
t163 = Iyz3.*dq1.*t54.*-5.0e-1;
t164 = Iyz3.*dq1.*t54.*5.0e-1;
t165 = Iyz3.*dq2.*t54.*-2.5e-1;
t166 = Iyz3.*dq2.*t54.*2.5e-1;
t167 = Iyz3.*dq2.*t54.*5.0e-1;
t168 = Iyz3.*dq3.*t54.*-2.5e-1;
t169 = Iyz3.*dq3.*t54.*2.5e-1;
t170 = Iyz3.*dq3.*t54.*5.0e-1;
t171 = cos(t125);
t172 = Ixy3.*dq2.*t55.*-2.5e-1;
t173 = Ixy3.*dq2.*t55.*2.5e-1;
t174 = Ixy3.*dq2.*t55.*5.0e-1;
t175 = Ixy3.*dq3.*t55.*-2.5e-1;
t176 = Ixy3.*dq3.*t55.*2.5e-1;
t177 = Ixy3.*dq3.*t55.*5.0e-1;
t178 = sin(t125);
t179 = Ixx3.*dq2.*t56.*-1.25e-1;
t180 = Ixx3.*dq2.*t56.*1.25e-1;
t181 = Ixx3.*dq2.*t56.*2.5e-1;
t182 = Ixx3.*dq3.*t56.*-1.25e-1;
t183 = Ixx3.*dq3.*t56.*1.25e-1;
t184 = Ixx3.*dq3.*t56.*2.5e-1;
t185 = Iyy3.*dq2.*t56.*1.25e-1;
t186 = Iyy3.*dq2.*t56.*-2.5e-1;
t187 = Iyy3.*dq2.*t56.*2.5e-1;
t188 = Iyy3.*dq3.*t56.*1.25e-1;
t189 = Iyy3.*dq3.*t56.*-2.5e-1;
t190 = Iyy3.*dq3.*t56.*2.5e-1;
t191 = Ixy3.*dq1.*t55.*-7.5e-1;
t192 = Ixy3.*dq1.*t55.*7.5e-1;
t193 = Ixy3.*dq1.*t55.*1.5;
t194 = Ixx3.*dq1.*t56.*-3.75e-1;
t195 = Ixx3.*dq1.*t56.*3.75e-1;
t196 = Ixx3.*dq1.*t56.*7.5e-1;
t197 = Iyy3.*dq1.*t56.*3.75e-1;
t198 = Iyy3.*dq1.*t56.*-7.5e-1;
t199 = Iyy3.*dq1.*t56.*7.5e-1;
t200 = Ixy3.*t88.*-2.5e-1;
t201 = Ixy3.*t88.*2.5e-1;
t202 = Ixx3.*t98.*1.25e-1;
t203 = Iyy3.*t98.*-1.25e-1;
t204 = Iyy3.*t98.*1.25e-1;
t205 = cos(t110);
t207 = Ixx2.*dq2.*t96.*2.5e-1;
t208 = Iyy2.*dq2.*t96.*2.5e-1;
t209 = Izz2.*dq2.*t96.*5.0e-1;
t236 = dq1.*m3.*t19.*t98.*5.0e-1;
t237 = dq1.*m3.*t19.*t98.*-1.0;
t238 = dq1.*m3.*t19.*t98.*1.0;
t285 = t41+t67+t100;
t286 = t42+t65+t105;
t287 = t33+t66+t107;
t288 = t34+t68+t103;
t206 = Ixz3.*t99.*5.0e-1;
t210 = Iyz3.*t102.*-5.0e-1;
t211 = Iyz3.*t102.*5.0e-1;
t212 = Ixz3.*dq2.*t99.*-2.5e-1;
t213 = Ixz3.*dq2.*t99.*2.5e-1;
t215 = Ixz3.*dq3.*t99.*-2.5e-1;
t216 = Ixz3.*dq3.*t99.*2.5e-1;
t218 = Iyz3.*dq2.*t102.*2.5e-1;
t221 = Iyz3.*dq3.*t102.*2.5e-1;
t224 = Ixy3.*dq2.*t205;
t225 = Ixy3.*dq3.*t205;
t226 = Iyz3.*t178.*5.0e-1;
t227 = Ixy3.*t205.*2.5e-1;
t228 = Ixx3.*t143.*2.5e-1;
t229 = Ixx3.*t144.*1.25e-1;
t230 = Iyy3.*t143.*2.5e-1;
t231 = Iyy3.*t144.*-1.25e-1;
t232 = Iyy3.*t144.*1.25e-1;
t233 = Izz3.*t143.*-5.0e-1;
t234 = Izz3.*t143.*5.0e-1;
t235 = Ixz3.*t171.*5.0e-1;
t243 = Ixx3.*dq2.*t143.*-2.5e-1;
t245 = Ixx3.*dq2.*t143.*5.0e-1;
t246 = Ixx3.*dq2.*t144.*-2.5e-1;
t247 = Ixx3.*dq2.*t144.*2.5e-1;
t248 = Ixx3.*dq3.*t143.*-2.5e-1;
t250 = Ixx3.*dq2.*t144.*5.0e-1;
t251 = Ixx3.*dq3.*t143.*5.0e-1;
t252 = Ixx3.*dq3.*t144.*-2.5e-1;
t253 = Ixx3.*dq3.*t144.*2.5e-1;
t254 = Ixx3.*dq3.*t144.*5.0e-1;
t255 = Iyy3.*dq2.*t143.*-2.5e-1;
t257 = Iyy3.*dq2.*t143.*5.0e-1;
t258 = Iyy3.*dq2.*t144.*2.5e-1;
t259 = Iyy3.*dq3.*t143.*-2.5e-1;
t261 = Iyy3.*dq2.*t144.*-5.0e-1;
t262 = Iyy3.*dq2.*t144.*5.0e-1;
t263 = Iyy3.*dq3.*t143.*5.0e-1;
t264 = Iyy3.*dq3.*t144.*2.5e-1;
t265 = Iyy3.*dq3.*t144.*-5.0e-1;
t266 = Iyy3.*dq3.*t144.*5.0e-1;
t268 = Izz3.*dq2.*t143.*-1.0;
t269 = Izz3.*dq2.*t143.*1.0;
t271 = Izz3.*dq3.*t143.*-1.0;
t272 = Izz3.*dq3.*t143.*1.0;
t273 = Ixz3.*dq2.*t171.*-7.5e-1;
t274 = Ixz3.*dq2.*t171.*7.5e-1;
t275 = Ixz3.*dq2.*t171.*1.5;
t276 = Ixz3.*dq3.*t171.*-7.5e-1;
t277 = Ixz3.*dq3.*t171.*7.5e-1;
t278 = Ixz3.*dq3.*t171.*1.5;
t279 = Iyz3.*dq2.*t178.*-7.5e-1;
t280 = Iyz3.*dq2.*t178.*7.5e-1;
t281 = Iyz3.*dq2.*t178.*1.5;
t282 = Iyz3.*dq3.*t178.*-7.5e-1;
t283 = Iyz3.*dq3.*t178.*7.5e-1;
t284 = Iyz3.*dq3.*t178.*1.5;
t289 = t4.*t285;
t290 = t2.*t288;
t291 = t2.*t286.*1.0;
t292 = t4.*t287.*-1.0;
t293 = t4.*t287.*1.0;
t214 = dq2.*t206;
t217 = dq3.*t206;
t219 = dq2.*t210;
t220 = dq2.*t211;
t222 = dq3.*t210;
t223 = dq3.*t211;
t239 = t224.*-5.0e-1;
t240 = t224.*5.0e-1;
t241 = t225.*-5.0e-1;
t242 = t225.*5.0e-1;
t244 = dq2.*t228;
t249 = dq3.*t228;
t256 = dq2.*t230;
t260 = dq3.*t230;
t267 = dq2.*t234;
t270 = dq3.*t234;
t294 = -t289;
t295 = t290+t292;
t298 = t2.*(t289-t291).*-5.0e-1;
t296 = t291+t294;
t297 = t4.*t295.*5.0e-1;
t299 = t297+t298;
t300 = dq1.*t299.*1.0;
et1 = t59+t62+t69+t72+t75+t78+t81+t84+t91+t94+t111+t115+t116+t119+t124+t126+t131+t134+t135+t139+t142+t145+t149+t152+t154+t159+t162+t163+t167+t170+t174+t177+t181+t184+t186+t189+t191+t194+t197-t207-t208+t209+t212+t215+t218+t221+t237+t239+t241+t243+t246+t248+t252+t255+t258+t259+t264+t267+t270+t273+t276+t279+t282+Ixx2.*dq2.*t21.*2.5e-1+Ixx2.*dq2.*t37.*2.5e-1-Ixx2.*dq1.*t45.*3.75e-1+Ixx2.*dq2.*t45.*2.5e-1-Ixx2.*dq2.*t97.*2.5e-1+Ixy2.*dq2.*t20.*5.0e-1+Ixy2.*dq2.*t30.*5.0e-1-Ixy2.*dq1.*t43.*7.5e-1+Ixy2.*dq2.*t43.*5.0e-1-Ixy2.*dq2.*t87.*5.0e-1-Ixz2.*dq2.*t3.*5.0e-1-Ixz2.*dq1.*t32.*5.0e-1-Ixz2.*dq2.*t31.*2.5e-1-Ixz2.*dq2.*t63.*7.5e-1-Iyy2.*dq2.*t21.*2.5e-1-Iyy2.*dq2.*t37.*2.5e-1+Iyy2.*dq1.*t45.*3.75e-1-Iyy2.*dq2.*t45.*2.5e-1+Iyy2.*dq2.*t97.*2.5e-1+Iyz2.*dq2.*t5.*5.0e-1-Iyz2.*dq1.*t39.*5.0e-1+Iyz2.*dq2.*t38.*2.5e-1-Iyz2.*dq2.*t64.*7.5e-1-dq1.*m2.*t18.*t22.*1.0-dq1.*m3.*t17.*t22.*1.0;
et2 = L2.*Lcom3.*dq1.*m3.*t40.*-2.0;
et3 = t57+t60+t70+t73+t76+t79+t83+t86+t90+t93+t112+t114+t117+t121+t122+t127+t129+t132+t137+t140+t147+t148+t151+t155+t157+t160+t165+t168+t172+t175+t179+t182+t185+t188+t193+t196+t198+t214+t217+t219+t222+t224+t225+t236+t245+t250+t251+t254+t257+t261+t263+t265+t268+t271+t275+t278+t281+t284-Ixx2.*dq2.*t21.*5.0e-1-Ixx2.*dq1.*t37.*2.5e-1+Ixx2.*dq1.*t45.*7.5e-1-Ixx2.*dq2.*t45.*1.25e-1+Ixx2.*dq2.*t96.*5.0e-1+Ixx2.*dq2.*t97.*5.0e-1-Ixy2.*dq2.*t20.*1.0-Ixy2.*dq1.*t30.*5.0e-1+Ixy2.*dq1.*t43.*1.5-Ixy2.*dq2.*t43.*2.5e-1+Ixy2.*dq2.*t87+Ixz2.*dq2.*t3.*2.5e-1+Ixz2.*dq1.*t32.*1.0-Ixz2.*dq2.*t32.*2.5e-1+Ixz3.*dq1.*t52.*1.0+Ixz2.*dq2.*t63.*1.5+Iyy2.*dq2.*t21.*5.0e-1+Iyy2.*dq1.*t37.*2.5e-1-Iyy2.*dq1.*t45.*7.5e-1+Iyy2.*dq2.*t45.*1.25e-1+Iyy2.*dq2.*t96.*5.0e-1-Iyy2.*dq2.*t97.*5.0e-1-Iyz2.*dq2.*t5.*2.5e-1+Iyz2.*dq1.*t39.*1.0-Iyz2.*dq2.*t39.*2.5e-1+Iyz3.*dq1.*t54.*1.0+Iyz2.*dq2.*t64.*1.5-Izz2.*dq2.*t96.*1.0;
et4 = dq1.*m2.*t18.*t22.*5.0e-1+dq1.*m3.*t17.*t22.*5.0e-1+L2.*Lcom3.*dq1.*m3.*t40;
mt1 = [t112+t115+t117+t121+t124+t129+t132+t139+t142+t148+t151+t157+t160+t167+t170+Ixx2.*dq1.*t21.*5.0e-1+Ixx3.*dq1.*t21.*5.0e-1+Ixx2.*dq2.*t45.*3.75e-1+Ixx3.*dq2.*t56.*3.75e-1+Ixx3.*dq3.*t56.*3.75e-1+Ixy2.*dq1.*t20+Ixy3.*dq1.*t20+Ixy2.*dq2.*t43.*7.5e-1+Ixy3.*dq2.*t55.*7.5e-1+Ixy3.*dq3.*t55.*7.5e-1-Iyy2.*dq1.*t21.*5.0e-1-Iyy3.*dq1.*t21.*5.0e-1-Iyy2.*dq2.*t45.*3.75e-1-Iyy3.*dq2.*t56.*3.75e-1-Iyy3.*dq3.*t56.*3.75e-1,et3+et4,t57+t60+t70+t73+t76+t79+t83+t86+t90+t93+t127+t129+t132+t137+t140+t147+t148+t151+t155+t157+t160+t165+t168+t172+t175+t179+t182+t185+t188+t193+t196+t198+t214+t217+t219+t222+t224+t225+t236+t245+t250+t251+t254+t257+t261+t263+t265+t268+t271+t275+t278+t281+t284+dq1.*t49+Ixz3.*dq1.*t52+Iyz3.*dq1.*t54+L2.*Lcom3.*dq1.*m3.*t40.*5.0e-1,et1+et2];
mt2 = [t111+t114+t116+t119+t122+t126+t145+t154+t207+t208-t209+t214+t217+t219+t222+t224.*2.5e-1+t225.*2.5e-1+t244+t249+t256+t260-dq1.*t33.*2.5e-1+dq1.*t42.*2.5e-1+dq2.*t200+dq3.*t200+dq2.*t202+dq2.*t203+dq3.*t202+dq3.*t203+dq2.*t226+dq3.*t226+dq2.*t229+dq3.*t229+dq2.*t231+dq3.*t231+dq2.*t233+dq3.*t233+dq2.*t235+dq3.*t235+Ixx2.*dq2.*t22.*1.25e-1+Ixx2.*dq1.*t45.*1.25e-1+Ixx3.*dq1.*t56.*1.25e-1+Ixx2.*dq2.*t97.*1.25e-1+Ixy2.*dq1.*t43.*2.5e-1+Ixy3.*dq1.*t55.*2.5e-1+Ixy2.*dq2.*t87.*2.5e-1-Ixz2.*dq1.*t3.*2.5e-1+Ixz2.*dq1.*t32.*2.5e-1+Ixz3.*dq1.*t52.*2.5e-1+Ixz2.*dq2.*t63.*5.0e-1-Iyy2.*dq2.*t22.*1.25e-1-Iyy2.*dq1.*t45.*1.25e-1-Iyy3.*dq1.*t56.*1.25e-1-Iyy2.*dq2.*t97.*1.25e-1+Iyz2.*dq1.*t5.*2.5e-1+Iyz2.*dq1.*t39.*2.5e-1+Iyz3.*dq1.*t54.*2.5e-1+Iyz2.*dq2.*t64.*5.0e-1-Ixy2.*dq2.*cos(t13).*2.5e-1];
mt3 = [t300+dq3.*(t49+t200+t202+t203+t206+t210+t226+t227+t228+t229+t230+t231+t233+t235)+dq2.*(t200+t202+t203+t206+t210+t226+t227+t228+t229+t230+t231+t233+t235+L2.*Lcom3.*m3.*t6),t59+t62+t69+t72+t75+t78+t81+t84+t91+t94+t126+t131+t134+t135+t139+t142+t145+t149+t152+t154+t159+t162+t163+t167+t170+t174+t177+t181+t184+t186+t189+t191+t194+t197+t212+t215+t218+t221+t237+t239+t241+t243+t246+t248+t252+t255+t258+t259+t264+t267+t270+t273+t276+t279+t282-L2.*Lcom3.*dq1.*m3.*t6.*1.0-L2.*Lcom3.*dq1.*m3.*t40.*1.0,t300+dq2.*(t200+t202+t203+t206+t210+t226+t227+t228+t229+t230+t231+t233+t235-L2.*Lcom3.*m3.*t6.*2.0)+dq3.*(t200+t202+t203+t206+t210+t226+t227+t228+t229+t230+t231+t233+t235-L2.*Lcom3.*m3.*t6.*1.0),t300+dq3.*(t200+t202+t203+t206+t210+t226+t227+t228+t229+t230+t231+t233+t235)+dq2.*(-t49+t200+t202+t203+t206+t210+t226+t227+t228+t229+t230+t231+t233+t235)];
C = reshape([mt1,mt2,mt3],3,3);
