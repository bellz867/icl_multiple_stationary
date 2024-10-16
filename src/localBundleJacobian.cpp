// jacobian for reprojection into key frame or generally, any previous frame
// minimize || f(x) - b ||^2 is equivalent to minimize f(x)^T*f(x)-2*b^T*f(x)
// find dEdx = 0 is equivalent to 2*J^T*f(x)-2*J^T*b = 0 where J = df(x)/dx
//expand around x f(x+Dx) = f(x)+J*Dx where Dx = (J^T*J)^-1*J^T*(b-f(x))

r11 = 1.0-2.0*(qy*qy+qz*qz);
r12 = 2.0*(qx*qy-qz*qw);
r13 = 2.0*(qx*qz+qy*qw);
r21 = 2.0*(qx*qy+qz*qw);
r22 = 1.0-2.0*(qx*qx+qz*qz);
r23 = 2.0*(qy*qz-qx*qw);
r31 = 2.0*(qw*qz-qy*qw);
r32 = 2.0*(qy*qz+qx*qw);
r33 = 1.0-2.0*(qx*qx+qy*qy);
xic = zic*(uic-cx)/fx;
yic = zic*(vic-cy)/fy;
xik = r11*(xic-xkc)+r21*(yic-ykc)+r31*(zic-zkc);
yik = r12*(xic-xkc)+r22*(yic-ykc)+r32*(zic-zkc);
zik = r13*(xic-xkc)+r23*(yic-ykc)+r33*(zic-zkc);
dukduc = fx*(zik*r11*zic/fx-xik*r13*zic/fx)/(zik*zik);
dukdvc = fx*(zik*r21*zic/fy-xik*r23*zic/fy)/(zik*zik);
dukdzc = fx*(zik*(r11*(uic-cx)/fx+r21*(vic-cy)/fy+r31)-xik*(r13*(uic-cx)/fx+r23*(vic-cy)/fy+r33))/(zik*zik);
dukdqw = fx*(zik*((2.0*qz*(zic*(vic-cy)/fy-ykc))+2.0*(qz-qy)*(zic-zkc))-xik*(2.0*qy*(zic*(uic-cx)/fx-xkc)-2.0*qx*(zic*(vic-cy)/fy-ykc)))/(zik*zik);
dukdqx = fx*(zik*(2.0*qy*(zic*(vic-cy)/fy-ykc))-xik*(2.0*qz*(zic*(uic-cx)/fx-xkc)-2.0*qw*(zic*(vic-cy)/fy-ykc)-4.0*qx*(zic-zkc)))/(zik*zik);
dukdqy = fx*(zik*(-4.0*qy*(zic*(uic-cx)/fx-xkc)+2.0*qx*(zic*(vic-cy)/fy-ykc)-2.0*qw*(zic-zkc))-xik*(2.0*qw*(zic*(uic-cx)/fx-xkc)+2.0*qz*(zic*(vic-cy)/fy-ykc)-4.0*qy*(zic-zkc)))/(zik*zik);
dukdqz = fx*(zik*(-4.0*qz*(zic*(uic-cx)/fx-xkc)+2.0*qw*(zic*(vic-cy)/fy-ykc)+2.0*qw*(zic-zkc))-xik*(2.0*qx*(zic*(uic-cx)/fx-xkc)+2.0*qy*(zic*(vic-cy)/fy-ykc)))/(zik*zik);
dukdxkc = fx*(-r11*zik+r13*xik)/(zik*zik);
dukdykc = fx*(-r21*zik+r23*xik)/(zik*zik);
dukdzkc = fx*(-r31*zik+r33*xik)/(zik*zik);

dvkduc = fy*(zik*r12*zic/fx-yik*r13*zic/fx)/(zik*zik);
dvkdvc = fy*(zik*r22*zic/fy-yik*r23*zic/fy)/(zik*zik);
dvkdzc = fy*(zik*(r12*(uic-cx)/fx+r22*(vic-cy)/fy+r32)-yik*(r13*(uic-cx)/fx+r23*(vic-cy)/fy+r33))/(zik*zik);
dvkdqw = fy*(zik*((-2.0*qz*(zic*(uic-cx)/fx-xkc))+2.0*qx*(zic-zkc))-yik*(2.0*qy*(zic*(uic-cx)/fx-xkc)-2.0*qx*(zic*(vic-cy)/fy-ykc)))/(zik*zik);
dvkdqx = fy*(zik*(2.0*qy*(zic*(uic-cx)/fx-xkc)-4.0*qx*(zic*(vic-cy)/fy-ykc)+2.0*qw*(zic-zkc))-yik*(2.0*qz*(zic*(uic-cx)/fx-xkc)-2.0*qw*(zic*(vic-cy)/fy-ykc)-4.0*qx*(zic-zkc)))/(zik*zik);
dvkdqy = fy*(zik*(2.0*qx*(zic*(uic-cx)/fx-xkc)+2.0*qz*(zic-zkc))-yik*(2.0*qw*(zic*(uic-cx)/fx-xkc)+2.0*qz*(zic*(vic-cy)/fy-ykc)-4.0*qy*(zic-zkc)))/(zik*zik);
dvkdqz = fy*(zik*(-2.0*qw*(zic*(uic-cx)/fx-xkc)-4.0*qz*(zic*(vic-cy)/fy-ykc)+2.0*qy*(zic-zkc))-yik*(2.0*qx*(zic*(uic-cx)/fx-xkc)+2.0*qy*(zic*(vic-cy)/fy-ykc)))/(zik*zik);
dvkdxkc = fy*(-r12*zik+r13*yik)/(zik*zik);
dvkdykc = fy*(-r22*zik+r23*yik)/(zik*zik);
dvkdzkc = fy*(-r32*zik+r33*yik)/(zik*zik);
