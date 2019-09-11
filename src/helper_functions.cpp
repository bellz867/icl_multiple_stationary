#include <helper_functions.h>

Eigen::Matrix3f getqRot(Eigen::Vector4f q)
{
	Eigen::Matrix3f R;
	R << 1.0-2.0*(q(2)*q(2)+q(3)*q(3)),     2.0*(q(1)*q(2)-q(3)*q(0)),     2.0*(q(1)*q(3)+q(2)*q(0)),
	         2.0*(q(1)*q(2)+q(3)*q(0)), 1.0-2.0*(q(1)*q(1)+q(3)*q(3)),     2.0*(q(2)*q(3)-q(1)*q(0)),
			     2.0*(q(0)*q(3)-q(2)*q(0)),     2.0*(q(2)*q(3)+q(1)*q(0)), 1.0-2.0*(q(1)*q(1)+q(2)*q(2));
  return R;
}

//differential q matrix
Eigen::Matrix<float,4,3> B(Eigen::Vector4f q)
{
	Eigen::Matrix<float,4,3> qDiff;
	qDiff << -q(1), -q(2), -q(3),
			  q(0), -q(3),  q(2),
			  q(3),  q(0), -q(1),
			 -q(2),  q(1),  q(0);
	return qDiff;
}

//q as matrix
Eigen::Matrix4f getqMat(Eigen::Vector4f q)
{
	Eigen::Matrix4f qMat;
	qMat << q(0), -q(1), -q(2), -q(3),
					q(1),  q(0), -q(3),  q(2),
					q(2),  q(3),  q(0), -q(1),
					q(3), -q(2),  q(1),  q(0);
	return qMat;
}

//q invers
Eigen::Vector4f getqInv(Eigen::Vector4f q)
{
	Eigen::Vector4f qInv;
	qInv << q(0), -q(1), -q(2), -q(3);
	return qInv;
}

//skew symmetric
Eigen::Matrix3f getss(Eigen::Vector3f p)
{
	Eigen::Matrix3f pss;
	pss <<   0.0, -p(2),  p(1),
	        p(2),   0.0, -p(0),
		   -p(1),  p(0),   0.0;
	return pss;
}

// rotate a vector
Eigen::Vector3f rotatevec(Eigen::Vector3f v, Eigen::Vector4f q)
{
	Eigen::Vector4f rotatedvec = getqMat(getqMat(q)*Eigen::Vector4f(0.0,v(0),v(1),v(2)))*getqInv(q);
	return rotatedvec.segment(1,3);
}

// rotate a vector
Eigen::Vector3f rotatevec(Eigen::Vector3f v, Eigen::Vector4f q, bool normalize)
{
	Eigen::Vector4f rotatedvec = getqMat(getqMat(q)*Eigen::Vector4f(0.0,v(0),v(1),v(2)))*getqInv(q);

	return rotatedvec.segment(1,3)/rotatedvec.segment(1,3).norm();
}

// distance derivative wrt time
float fd(Eigen::Vector3f u, Eigen::Vector3f vc)
{
	return -1.0*u.transpose()*vc;
}

// unit vector derivative wrt time
Eigen::Vector3f fu(Eigen::Vector3f u, float d, Eigen::Vector3f vc, Eigen::Vector3f wc)
{
	return (1.0/d*(u*u.transpose() - Eigen::Matrix3f::Identity())*vc - getss(wc)*u);
}

// quaternion derivative wrt time
Eigen::Vector4f fq(Eigen::Vector4f q, Eigen::Vector3f w)
{
	return 0.5*B(q)*w;
}

// position derivative wrt time
Eigen::Vector3f fp(Eigen::Vector4f q, Eigen::Vector3f v)
{
	return rotatevec(v,q);
}

// guic
Eigen::Matrix3f guic(Eigen::Vector3f u, Eigen::Vector3f v)
{
	float u1 = u(0);
	float u2 = u(1);
	float u3 = u(2);
	float v1 = v(0);
	float v2 = v(1);
	float v3 = v(2);
	Eigen::Matrix3f gu;
	gu << 2.0*u1*v1+u2*v2+u3*v3,                 u1*v2,                 u1*v3,
						  u2*v1, u1*v1+2.0*u2*v2+u3*v3,                 u2*v3,
						  u3*v1,                 u3*v2, u1*v1+u2*v2+2.0*u3*v3;
	return gu;
}

// gpck
Eigen::Matrix<float,3,4> gpck(Eigen::Vector4f q, Eigen::Vector3f v)
{
	float qw = q(0);
	float qx = q(1);
	float qy = q(2);
	float qz = q(3);
	float vx = v(0);
	float vy = v(1);
	float vz = v(2);
	Eigen::Matrix<float,3,4> gp;
	gp << 2.0*qw*vx+2.0*qy*vz-2.0*qz*vy, 2.0*qx*vx+2.0*qy*vy+2.0*qz*vz, 2.0*qw*vz+2.0*qx*vy-2.0*qy*vx, 2.0*qx*vz-2.0*qw*vy-2.0*qz*vx,
		  2.0*qw*vy-2.0*qx*vz+2.0*qz*vx, 2.0*qy*vx-2.0*qx*vy-2.0*qw*vz, 2.0*qx*vx+2.0*qy*vy+2.0*qz*vz, 2.0*qw*vx+2.0*qy*vz-2.0*qz*vy,
		  2.0*qw*vz+2.0*qx*vy-2.0*qy*vx, 2.0*qw*vy-2.0*qx*vz+2.0*qz*vx, 2.0*qz*vy-2.0*qy*vz-2.0*qw*vx, 2.0*qx*vx+2.0*qy*vy+2.0*qz*vz;
	return gp;
}

//gqck
Eigen::Matrix4f gqck(Eigen::Vector3f wc)
{
	float wx = wc(0);
	float wy = wc(1);
	float wz = wc(2);
	Eigen::Matrix4f gq;
	gq <<       0.0, -wx/2.0, -wy/2.0, -wz/2.0,
			 wx/2.0,     0.0,  wz/2.0, -wy/2.0,
			 wy/2.0, -wz/2.0,     0.0,  wx/2.0,
			 wz/2.0,  wy/2.0, -wx/2.0,     0.0;
	return gq;
}

// gcuic
Eigen::Matrix3f gcuic(Eigen::Vector4f q)
{
	float qw = q(0);
	float qx = q(1);
	float qy = q(2);
	float qz = q(3);
	float gcu11 = std::pow(qw,2.0)+std::pow(qx,2.0)-std::pow(qy,2.0)-std::pow(qz,2.0);
	float gcu22 = std::pow(qw,2.0)-std::pow(qx,2.0)+std::pow(qy,2.0)-std::pow(qz,2.0);
	float gcu33 = std::pow(qw,2.0)-std::pow(qx,2.0)-std::pow(qy,2.0)+std::pow(qz,2.0);
	Eigen::Matrix3f gcu;
	gcu <<	              gcu11, 2.0*qx*qy-2.0*qw*qz, 2.0*qw*qy+2.0*qx*qz,
			2.0*qw*qz+2.0*qx*qy,               gcu22, 2.0*qy*qz-2.0*qw*qx,
			2.0*qx*qz-2.0*qw*qy, 2.0*qw*qx+2.0*qy*qz,               gcu33;
	return gcu;
}

//gcqck
Eigen::Matrix<float,3,4> gcqck(Eigen::Vector4f q, Eigen::Vector3f p)
{
	return gpck(q,p);
}



//bundle adjust jacobian
Eigen::Vector3f localBundleProjection(float fx, float fy, float cx, float cy, float uic, float vic, float zic, Eigen::Vector4f q, Eigen::Vector3f p, bool usezik)
{
	// reprojection into key frame or generally, any previous frame
	float xkc = p(0);
	float ykc = p(1);
	float zkc = p(2);
	float qw = q(0);
	float qx = q(1);
	float qy = q(2);
	float qz = q(3);
	float r11 = 1.0-2.0*(qy*qy+qz*qz);
	float r12 = 2.0*(qx*qy-qz*qw);
	float r13 = 2.0*(qx*qz+qy*qw);
	float r21 = 2.0*(qx*qy+qz*qw);
	float r22 = 1.0-2.0*(qx*qx+qz*qz);
	float r23 = 2.0*(qy*qz-qx*qw);
	float r31 = 2.0*(qw*qz-qy*qw);
	float r32 = 2.0*(qy*qz+qx*qw);
	float r33 = 1.0-2.0*(qx*qx+qy*qy);
	float xic = zic*(uic-cx)/fx;
	float yic = zic*(vic-cy)/fy;
	float xik = r11*(xic-xkc)+r21*(yic-ykc)+r31*(zic-zkc);
	float yik = r12*(xic-xkc)+r22*(yic-ykc)+r32*(zic-zkc);
	float zik = r13*(xic-xkc)+r23*(yic-ykc)+r33*(zic-zkc);

	Eigen::Vector3f projectioni(fx*xik/zik+cx,fy*yik/zik+cy,zik);
	return projectioni;
}

//bundle adjust jacobian
Eigen::Vector2f localBundleProjection(float fx, float fy, float cx, float cy, float uic, float vic, float zic, Eigen::Vector4f q, Eigen::Vector3f p)
{
	// reprojection into key frame or generally, any previous frame
	float xkc = p(0);
	float ykc = p(1);
	float zkc = p(2);
	float qw = q(0);
	float qx = q(1);
	float qy = q(2);
	float qz = q(3);
	float r11 = 1.0-2.0*(qy*qy+qz*qz);
	float r12 = 2.0*(qx*qy-qz*qw);
	float r13 = 2.0*(qx*qz+qy*qw);
	float r21 = 2.0*(qx*qy+qz*qw);
	float r22 = 1.0-2.0*(qx*qx+qz*qz);
	float r23 = 2.0*(qy*qz-qx*qw);
	float r31 = 2.0*(qw*qz-qy*qw);
	float r32 = 2.0*(qy*qz+qx*qw);
	float r33 = 1.0-2.0*(qx*qx+qy*qy);
	float xic = zic*(uic-cx)/fx;
	float yic = zic*(vic-cy)/fy;
	float xik = r11*(xic-xkc)+r21*(yic-ykc)+r31*(zic-zkc);
	float yik = r12*(xic-xkc)+r22*(yic-ykc)+r32*(zic-zkc);
	float zik = r13*(xic-xkc)+r23*(yic-ykc)+r33*(zic-zkc);

	Eigen::Vector2f projectioni(fx*xik/zik+cx,fy*yik/zik+cy);
	return projectioni;
}

// //bundle adjust jacobian
// Eigen::Matrix<float,3,10> localBundleJacobian(float fx, float fy, float cx, float cy, float uic, float vic, float zic, Eigen::Vector4f q, Eigen::Vector3f p, bool usezik)
// {
// 	Eigen::Matrix<float,3,10> Ji;
// 	Ji.block(0,0,2,10) = localBundleJacobian(fx,fy,cx,cy,uic,vic,zic,q,p);
// 	float xkc = p(0);
// 	float ykc = p(1);
// 	float zkc = p(2);
// 	float qw = q(0);
// 	float qx = q(1);
// 	float qy = q(2);
// 	float qz = q(3);
// 	float r13 = 2.0*(qx*qz+qy*qw);
// 	float r23 = 2.0*(qy*qz-qx*qw);
// 	float r33 = 1.0-2.0*(qx*qx+qy*qy);
// 	float xic = zic*(uic-cx)/fx;
// 	float yic = zic*(vic-cy)/fy;
// 	Ji(2,0) = r13*zic/fx;
// 	Ji(2,1) = r23*zic/fy;
// 	Ji(2,2) = r13*(uic-cx)/fx + r23*(vic-cy)/fy + r33;
// 	Ji(2,3) = 2.0*qy*(xic -xkc) - 2.0*qx*(yic-ykc);
// 	Ji(2,4) = 2.0*qz*(xic -xkc) - 2.0*qw*(yic-ykc) - 4.0*qx*(zic-zkc);
// 	Ji(2,5) = 2.0*qw*(xic -xkc) + 2.0*qz*(yic-ykc) - 4.0*qy*(zic-zkc);
// 	Ji(2,6) = 2.0*qx*(xic -xkc) + 2.0*qy*(yic-ykc);
// 	Ji(2,7) = -r13;
// 	Ji(2,8) = -r23;
// 	Ji(2,9) = -r33;
// 	return Ji;
// }
//
// //bundle adjust jacobian
// Eigen::Matrix<float,2,8> localBundleJacobian(float fx, float fy, float cx, float cy, float uic, float vic, float zic, Eigen::Vector4f q, Eigen::Vector3f p)
// {
// 	// jacobian for reprojection into key frame or generally, any previous frame
// 	// minimize || f(x) - b ||^2 is equivalent to minimize f(x)^T*f(x)-2*b^T*f(x)
// 	// find dEdx = 0 is equivalent to 2*J^T*f(x)-2*J^T*b = 0 where J = df(x)/dx
// 	//expand around x f(x+Dx) = f(x)+J*Dx where Dx = (J^T*J)^-1*J^T*(b-f(x))
// 	float xkc = p(0);
// 	float ykc = p(1);
// 	float zkc = p(2);
// 	float qw = q(0);
// 	float qx = q(1);
// 	float qy = q(2);
// 	float qz = q(3);
// 	float r11 = 1.0-2.0*(qy*qy+qz*qz);
// 	float r12 = 2.0*(qx*qy-qz*qw);
// 	float r13 = 2.0*(qx*qz+qy*qw);
// 	float r21 = 2.0*(qx*qy+qz*qw);
// 	float r22 = 1.0-2.0*(qx*qx+qz*qz);
// 	float r23 = 2.0*(qy*qz-qx*qw);
// 	float r31 = 2.0*(qw*qz-qy*qw);
// 	float r32 = 2.0*(qy*qz+qx*qw);
// 	float r33 = 1.0-2.0*(qx*qx+qy*qy);
// 	float xic = zic*(uic-cx)/fx;
// 	float yic = zic*(vic-cy)/fy;
// 	float xik = r11*(xic-xkc)+r21*(yic-ykc)+r31*(zic-zkc);
// 	float yik = r12*(xic-xkc)+r22*(yic-ykc)+r32*(zic-zkc);
// 	float zik = r13*(xic-xkc)+r23*(yic-ykc)+r33*(zic-zkc);
//
// 	//partials of uik
// 	float dukdzc = fx*(zik*(r11*(uic-cx)/fx+r21*(vic-cy)/fy+r31)-xik*(r13*(uic-cx)/fx+r23*(vic-cy)/fy+r33))/(zik*zik);
// 	float dukdqw = fx*(zik*((2.0*qz*(zic*(vic-cy)/fy-ykc))+2.0*(qz-qy)*(zic-zkc))-xik*(2.0*qy*(zic*(uic-cx)/fx-xkc)-2.0*qx*(zic*(vic-cy)/fy-ykc)))/(zik*zik);
// 	float dukdqx = fx*(zik*(2.0*qy*(zic*(vic-cy)/fy-ykc))-xik*(2.0*qz*(zic*(uic-cx)/fx-xkc)-2.0*qw*(zic*(vic-cy)/fy-ykc)-4.0*qx*(zic-zkc)))/(zik*zik);
// 	float dukdqy = fx*(zik*(-4.0*qy*(zic*(uic-cx)/fx-xkc)+2.0*qx*(zic*(vic-cy)/fy-ykc)-2.0*qw*(zic-zkc))-xik*(2.0*qw*(zic*(uic-cx)/fx-xkc)+2.0*qz*(zic*(vic-cy)/fy-ykc)-4.0*qy*(zic-zkc)))/(zik*zik);
// 	float dukdqz = fx*(zik*(-4.0*qz*(zic*(uic-cx)/fx-xkc)+2.0*qw*(zic*(vic-cy)/fy-ykc)+2.0*qw*(zic-zkc))-xik*(2.0*qx*(zic*(uic-cx)/fx-xkc)+2.0*qy*(zic*(vic-cy)/fy-ykc)))/(zik*zik);
// 	float dukdxkc = fx*(-r11*zik+r13*xik)/(zik*zik);
// 	float dukdykc = fx*(-r21*zik+r23*xik)/(zik*zik);
// 	float dukdzkc = fx*(-r31*zik+r33*xik)/(zik*zik);
//
// 	//partials of vik
// 	float dvkdzc = fy*(zik*(r12*(uic-cx)/fx+r22*(vic-cy)/fy+r32)-yik*(r13*(uic-cx)/fx+r23*(vic-cy)/fy+r33))/(zik*zik);
// 	float dvkdqw = fy*(zik*((-2.0*qz*(zic*(uic-cx)/fx-xkc))+2.0*qx*(zic-zkc))-yik*(2.0*qy*(zic*(uic-cx)/fx-xkc)-2.0*qx*(zic*(vic-cy)/fy-ykc)))/(zik*zik);
// 	float dvkdqx = fy*(zik*(2.0*qy*(zic*(uic-cx)/fx-xkc)-4.0*qx*(zic*(vic-cy)/fy-ykc)+2.0*qw*(zic-zkc))-yik*(2.0*qz*(zic*(uic-cx)/fx-xkc)-2.0*qw*(zic*(vic-cy)/fy-ykc)-4.0*qx*(zic-zkc)))/(zik*zik);
// 	float dvkdqy = fy*(zik*(2.0*qx*(zic*(uic-cx)/fx-xkc)+2.0*qz*(zic-zkc))-yik*(2.0*qw*(zic*(uic-cx)/fx-xkc)+2.0*qz*(zic*(vic-cy)/fy-ykc)-4.0*qy*(zic-zkc)))/(zik*zik);
// 	float dvkdqz = fy*(zik*(-2.0*qw*(zic*(uic-cx)/fx-xkc)-4.0*qz*(zic*(vic-cy)/fy-ykc)+2.0*qy*(zic-zkc))-yik*(2.0*qx*(zic*(uic-cx)/fx-xkc)+2.0*qy*(zic*(vic-cy)/fy-ykc)))/(zik*zik);
// 	float dvkdxkc = fy*(-r12*zik+r13*yik)/(zik*zik);
// 	float dvkdykc = fy*(-r22*zik+r23*yik)/(zik*zik);
// 	float dvkdzkc = fy*(-r32*zik+r33*yik)/(zik*zik);
//
// 	Eigen::Matrix<float,2,8> Ji;
// 	Ji << dukdzc,dukdqw,dukdqx,dukdqy,dukdqz,dukdxkc,dukdykc,dukdzkc,
// 	      dvkdzc,dvkdqw,dvkdqx,dvkdqy,dvkdqz,dvkdxkc,dvkdykc,dvkdzkc;
// 	return Ji;
// }

//bundle adjust jacobian
Eigen::Matrix<float,2,10> localBundleJacobian(Eigen::Vector3f pik, Eigen::Vector4f qkc, Eigen::Vector3f pkc)
{
	// jacobian for reprojection into key frame or generally, any previous frame
	// minimize || f(x) - b ||^2 is equivalent to minimize f(x)^T*f(x)-2*b^T*f(x)
	// find dEdx = 0 is equivalent to 2*J^T*f(x)-2*J^T*b = 0 where J = df(x)/dx
	//expand around x f(x+Dx) = f(x)+J*Dx where Dx = (J^T*J)^-1*J^T*(b-f(x))
	float xik = pik(0);
	float yik = pik(1);
	float zik = pik(2);
	float qw = qkc(0);
	float qx = qkc(1);
	float qy = qkc(2);
	float qz = qkc(3);
	float xkc = pkc(0);
	float ykc = pkc(1);
	float zkc = pkc(2);
	float r11 = 1.0-2.0*(qy*qy+qz*qz);
	float r12 = 2.0*(qx*qy-qz*qw);
	float r13 = 2.0*(qx*qz+qy*qw);
	float r21 = 2.0*(qx*qy+qz*qw);
	float r22 = 1.0-2.0*(qx*qx+qz*qz);
	float r23 = 2.0*(qy*qz-qx*qw);
	float r31 = 2.0*(qw*qz-qy*qw);
	float r32 = 2.0*(qy*qz+qx*qw);
	float r33 = 1.0-2.0*(qx*qx+qy*qy);
	float xic = xkc + r11*xik + r12*yik + r13*zik;
	float yic = ykc + r21*xik + r22*yik + r23*zik;
	float zic = zkc + r31*xik + r32*yik + r33*zik;
	float zic2 = zic*zic;
	float fxzz = fx*zic/zic2;
	float fxxz = fx*xic/zic2;
	float fyzz = fy*zic/zic2;
	float fyyz = fy*yic/zic2;


	Eigen::Matrix<float,2,10> Ji = Eigen::Matrix<float,2,10>::Zero();
	float dxdqwc = -2.0*qz*yik+2.0*qy*zik;
	float dxdqxc = 2.0*qy*yik+2.0*qz*zik;
	float dxdqyc = -4.0*qy*xik+2.0*qx*yik+2.0*qw*zik;
	float dxdqzc = -4.0*qz*xik-2.0*qw*yik+2.0*qx*zik;
	float dxdxc = 1.0;
	float dxdyc = 0.0;
	float dxdzc = 0.0;
	float dxdxi = r11;
	float dxdyi = r12;
	float dxdzi = r13;

	float dydqwc = 2.0*qz*xik-2.0*qx*zik;
	float dydqxc = 2.0*qy*xik-4.0*qx*yik-2.0*qw*zik;
	float dydqyc = 2.0*qx*xik+2.0*qz*zik;
	float dydqzc = 2.0*qw*xik-4.0*qz*yik+2.0*qy*zik;
	float dydxc = 0.0;
	float dydyc = 1.0;
	float dydzc = 0.0;
	float dydxi = r21;
	float dydyi = r22;
	float dydzi = r23;

	float dzdqwc = 2.0*(qz-qy)*xik+2.0*qx*yik;
	float dzdqxc = 2.0*qw*yik-4.0*qx*zik;
	float dzdqyc = -2.0*qw*xik+2.0*qz*yik-4.0*qy*zik;
	float dzdqzc = 2.0*qw*xik+2.0*qy*yik;
	float dzdxc = 0.0;
	float dzdyc = 0.0;
	float dzdzc = 1.0;
	float dzdxi = r31;
	float dzdyi = r32;
	float dzdzi = r33;

	Ji(0,0) = fxzz*dxdqwc-fxxz*dzdqwc;
	Ji(0,1) = fxzz*dxdqxc-fxxz*dzdqxc;
	Ji(0,2) = fxzz*dxdqyc-fxxz*dzdqyc;
	Ji(0,3) = fxzz*dxdqzc-fxxz*dzdqzc;
	Ji(0,4) = fxzz*dxdxc-fxxz*dzdxc;
	Ji(0,5) = fxzz*dxdyc-fxxz*dzdyc;
	Ji(0,6) = fxzz*dxdzc-fxxz*dzdzc;
	Ji(0,7) = fxzz*dxdxi-fxxz*dzdxi;
	Ji(0,8) = fxzz*dxdyi-fxxz*dzdyi;
	Ji(0,9) = fxzz*dxdzi-fxxz*dzdzi;

	Ji(1,0) = fyzz*dydqwc-fyyz*dzdqwc;
	Ji(2,1) = fyzz*dydqxc-fyyz*dzdqxc;
	Ji(1,2) = fyzz*dydqyc-fyyz*dzdqyc;
	Ji(1,3) = fyzz*dydqzc-fyyz*dzdqzc;
	Ji(1,4) = fyzz*dydxc-fyyz*dzdxc;
	Ji(1,5) = fyzz*dydyc-fyyz*dzdyc;
	Ji(1,6) = fyzz*dydzc-fyyz*dzdzc;
	Ji(1,7) = fyzz*dydxi-fyyz*dzdxi;
	Ji(1,8) = fyzz*dydyi-fyyz*dzdyi;
	Ji(1,9) = fyzz*dydzi-fyyz*dzdzi;

	return Ji;
}

//bundle adjust jacobian
Eigen::Matrix<float,3,7> localBundleJacobian(Eigen::Vector3f pik, Eigen::Vector4f qkc, float dkc)
{
	// jacobian for reprojection into key frame or generally, any previous frame
	// minimize || f(x) - b ||^2 is equivalent to minimize f(x)^T*f(x)-2*b^T*f(x)
	// find dEdx = 0 is equivalent to 2*J^T*f(x)-2*J^T*b = 0 where J = df(x)/dx
	//expand around x f(x+Dx) = f(x)+J*Dx where Dx = (J^T*J)^-1*J^T*(b-f(x))
	float xik = pik(0);
	float yik = pik(1);
	float zik = pik(2);
	float qw = qkc(0);
	float qx = qkc(1);
	float qy = qkc(2);
	float qz = qkc(3);

	Eigen::Matrix<float,3,7> Ji = Eigen::Matrix<float,3,7>::Zero();
	Ji(0,0) = -2.0*qz*yik+2.0*qy*zik;
	Ji(0,1) = 2.0*qy*yik+2.0*qz*zik;
	Ji(0,2) = -4.0*qy*xik+2.0*qx*yik+2.0*qw*zik;
	Ji(0,3) = -4.0*qz*xik-2.0*qw*yik+2.0*qx*zik;
	Ji(0,4) = dkc;
	Ji(1,0) = 2.0*qz*xik-2.0*qx*zik;
	Ji(1,1) = 2.0*qy*xik-4.0*qx*yik-2.0*qw*zik;
	Ji(1,2) = 2.0*qx*xik+2.0*qz*zik;
	Ji(1,3) = 2.0*qw*xik-4.0*qz*yik+2.0*qy*zik;
	Ji(1,5) = dkc;
	Ji(2,0) = 2.0*(qz-qy)*xik+2.0*qx*yik;
	Ji(2,1) = 2.0*qw*yik-4.0*qx*zik;
	Ji(2,2) = -2.0*qw*xik+2.0*qz*yik-4.0*qy*zik;
	Ji(2,3) = 2.0*qw*xik+2.0*qy*yik;
	Ji(2,6) = dkc;

	return Ji;
}

Eigen::Matrix<float,2,7> localBundleJacobianNorm(Eigen::Vector4f qkc, Eigen::Vector3f tkc)
{
	float xkc = tkc(0);
	float ykc = tkc(1);
	float zkc = tkc(2);
	float qw = qkc(0);
	float qx = qkc(1);
	float qy = qkc(2);
	float qz = qkc(3);

	Eigen::Matrix<float,2,7> Ji = Eigen::Matrix<float,2,7>::Zero();
	Ji(0,0) = 2.0*qw;
	Ji(0,1) = 2.0*qx;
	Ji(0,2) = 2.0*qy;
	Ji(0,3) = 2.0*qz;

	Ji(1,4) = 2.0*xkc;
	Ji(1,5) = 2.0*ykc;
	Ji(1,6) = 2.0*zkc;
	return Ji;
}

// Eigen::Vector2f localBundleJacobianPnPProject(float fx, float fy, float cx, float cy, Eigen::Vector3f pik, Eigen::Vector4f qkc, Eigen::Vector3f tkc, float dkc)
// {
// 	float xkc = tkc(0);
// 	float ykc = tkc(1);
// 	float zkc = tkc(2);
// 	float qw = qkc(0);
// 	float qx = qkc(1);
// 	float qy = qkc(2);
// 	float qz = qkc(3);
// 	float r11 = 1.0-2.0*(qy*qy+qz*qz);
// 	float r12 = 2.0*(qx*qy-qz*qw);
// 	float r13 = 2.0*(qx*qz+qy*qw);
// 	float r21 = 2.0*(qx*qy+qz*qw);
// 	float r22 = 1.0-2.0*(qx*qx+qz*qz);
// 	float r23 = 2.0*(qy*qz-qx*qw);
// 	float r31 = 2.0*(qw*qz-qy*qw);
// 	float r32 = 2.0*(qy*qz+qx*qw);
// 	float r33 = 1.0-2.0*(qx*qx+qy*qy);
// 	float xik = pik(0);
// 	float yik = pik(1);
// 	float zik = pik(2);
// 	float xic = xkc*dkc+r11*xik+r12*yik+r13*zik;
// 	float yic = ykc*dkc+r21*xik+r22*yik+r23*zik;
// 	float zic = zkc*dkc+r31*xik+r32*yik+r33*zik;
// 	float uic = fx*xic/zic+cx;
// 	float vic = fy*yic/zic+cy;
// 	return Eigen::Vector2f(uic,vic);
// }
//
// //bundle adjust jacobian
// Eigen::Matrix<float,2,7> localBundleJacobianPnP(float fx, float fy, Eigen::Vector3f pik, Eigen::Vector4f qkc, Eigen::Vector3f tkc, float dkc)
// {
// 	// jacobian for reprojection into key frame or generally, any previous frame
// 	// minimize || f(x) - b ||^2 is equivalent to minimize f(x)^T*f(x)-2*b^T*f(x)
// 	// find dEdx = 0 is equivalent to 2*J^T*f(x)-2*J^T*b = 0 where J = df(x)/dx
// 	//expand around x f(x+Dx) = f(x)+J*Dx where Dx = (J^T*J)^-1*J^T*(b-f(x))
// 	float xkc = tkc(0);
// 	float ykc = tkc(1);
// 	float zkc = tkc(2);
// 	float qw = qkc(0);
// 	float qx = qkc(1);
// 	float qy = qkc(2);
// 	float qz = qkc(3);
// 	float r11 = 1.0-2.0*(qy*qy+qz*qz);
// 	float r12 = 2.0*(qx*qy-qz*qw);
// 	float r13 = 2.0*(qx*qz+qy*qw);
// 	float r21 = 2.0*(qx*qy+qz*qw);
// 	float r22 = 1.0-2.0*(qx*qx+qz*qz);
// 	float r23 = 2.0*(qy*qz-qx*qw);
// 	float r31 = 2.0*(qw*qz-qy*qw);
// 	float r32 = 2.0*(qy*qz+qx*qw);
// 	float r33 = 1.0-2.0*(qx*qx+qy*qy);
// 	float xik = pik(0);
// 	float yik = pik(1);
// 	float zik = pik(2);
// 	float xic = xkc*dkc+r11*xik+r12*yik+r13*zik;
// 	float yic = ykc*dkc+r21*xik+r22*yik+r23*zik;
// 	float zic = zkc*dkc+r31*xik+r32*yik+r33*zik;
// 	float zic2 = zic*zic;
// 	float dudxi1 = fx*zic/zic2;
// 	float dudxi2 = fx*xic/zic2;
// 	float dvdxi1 = fy*zic/zic2;
// 	float dvdxi2 = fy*yic/zic2;
//
// 	//partials of x
// 	float dxdqw = -2.0*qz*yik+2.0*qy*zik;
// 	float dxdqx = 2.0*qy*yik+2.0*qz*zik;
// 	float dxdqy = -4.0*qy*xik+2.0*qx*yik+2.0*qw*zik;
// 	float dxdqz = -4.0*qz*xik-2.0*qw*yik+2.0*qx*zik;
// 	float dxdx = dkc;
// 	float dxdy = 0.0;
// 	float dxdz = 0.0;
//
// 	//partials of y
// 	float dydqw = 2.0*qz*xik-2.0*qx*zik;
// 	float dydqx = 2.0*qy*xik-4.0*qx*yik-2.0*qw*zik;
// 	float dydqy = 2.0*qx*xik+2.0*qz*zik;
// 	float dydqz = 2.0*qw*xik-4.0*qz*yik+2.0*qy*zik;
// 	float dydx = 0.0;
// 	float dydy = dkc;
// 	float dydz = 0.0;
//
// 	//partials of z
// 	float dzdqw = 2.0*(qz-qy)*xik+2.0*qx*yik;
// 	float dzdqx = 2.0*qw*yik-4.0*qx*zik;
// 	float dzdqy = -2.0*qw*xik+2.0*qz*yik-4.0*qy*zik;
// 	float dzdqz = 2.0*qw*xik+2.0*qy*yik;
// 	float dzdx = 0.0;
// 	float dzdy = 0.0;
// 	float dzdz = dkc;
//
// 	Eigen::Matrix<float,2,7> Ji;
// 	Ji << (dudxi1*dxdqw-dudxi2*dzdqw),(dudxi1*dxdqx-dudxi2*dzdqx),(dudxi1*dxdqy-dudxi2*dzdqy),(dudxi1*dxdqz-dudxi2*dzdqz),(dudxi1*dxdx-dudxi2*dzdx),(dudxi1*dxdy-dudxi2*dzdy),(dudxi1*dxdz-dudxi2*dzdz),
// 	      (dvdxi1*dydqw-dvdxi2*dzdqw),(dvdxi1*dydqx-dvdxi2*dzdqx),(dvdxi1*dydqy-dvdxi2*dzdqy),(dvdxi1*dydqz-dvdxi2*dzdqz),(dvdxi1*dydx-dvdxi2*dzdx),(dvdxi1*dydy-dvdxi2*dzdy),(dvdxi1*dydz-dvdxi2*dzdz);
// 	return Ji;
// }
//
// Eigen::Matrix<float,3,7> localBundleJacobian(Eigen::Vector3f pik, Eigen::Vector4f qkc, Eigen::Vector3f tkc, float dkc)
// {
// 	// jacobian for reprojection into key frame or generally, any previous frame
// 	// minimize || f(x) - b ||^2 is equivalent to minimize f(x)^T*f(x)-2*b^T*f(x)
// 	// find dEdx = 0 is equivalent to 2*J^T*f(x)-2*J^T*b = 0 where J = df(x)/dx
// 	//expand around x f(x+Dx) = f(x)+J*Dx where Dx = (J^T*J)^-1*J^T*(b-f(x))
// 	float xik = pik(0);
// 	float yik = pik(1);
// 	float zik = pik(2);
// 	float xkc = tkc(0);
// 	float ykc = tkc(1);
// 	float zkc = tkc(2);
// 	float qw = qkc(0);
// 	float qx = qkc(1);
// 	float qy = qkc(2);
// 	float qz = qkc(3);
// 	float r11 = 1.0-2.0*(qy*qy+qz*qz);
// 	float r12 = 2.0*(qx*qy-qz*qw);
// 	float r13 = 2.0*(qx*qz+qy*qw);
// 	float r21 = 2.0*(qx*qy+qz*qw);
// 	float r22 = 1.0-2.0*(qx*qx+qz*qz);
// 	float r23 = 2.0*(qy*qz-qx*qw);
// 	float r31 = 2.0*(qw*qz-qy*qw);
// 	float r32 = 2.0*(qy*qz+qx*qw);
// 	float r33 = 1.0-2.0*(qx*qx+qy*qy);
//
// 	Eigen::Matrix<float,3,7> Ji = Eigen::Matrix<float,3,7>::Zero();
// 	Ji(0,0) = -2.0*qz*yik+2.0*qy*zik;
// 	Ji(0,1) = 2.0*qy*yik+2.0*qz*zik;
// 	Ji(0,2) = -4.0*qy*xik+2.0*qx*yik+2.0*qw*zik;
// 	Ji(0,3) = -4.0*qz*xik-2.0*qw*yik+2.0*qx*zik;
// 	Ji(0,4) = dkc;
//
// 	Ji(1,0) = 2.0*qz*xik-2.0*qx*zik;
// 	Ji(1,1) = 2.0*qy*xik-4.0*qx*yik-2.0*qw*zik;
// 	Ji(1,2) = 2.0*qx*xik+2.0*qz*zik;
// 	Ji(1,3) = 2.0*qw*xik-4.0*qz*yik+2.0*qy*zik;
// 	Ji(1,5) = dkc;
//
// 	Ji(2,0) = 2.0*(qz-qy)*xik+2.0*qx*yik;
// 	Ji(2,1) = 2.0*qw*yik-4.0*qx*zik;
// 	Ji(2,2) = -2.0*qw*xik+2.0*qz*yik-4.0*qy*zik;
// 	Ji(2,3) = 2.0*qw*xik+2.0*qy*yik;
// 	Ji(2,6) = dkc;
// 	return Ji;
// }


// //bundle adjust jacobian
// Eigen::Matrix<float,2,10> localBundleJacobian(float fx, float fy, float cx, float cy, float uic, float vic, float zic, Eigen::Vector4f q, Eigen::Vector3f p)
// {
// 	// jacobian for reprojection into key frame or generally, any previous frame
// 	// minimize || f(x) - b ||^2 is equivalent to minimize f(x)^T*f(x)-2*b^T*f(x)
// 	// find dEdx = 0 is equivalent to 2*J^T*f(x)-2*J^T*b = 0 where J = df(x)/dx
// 	//expand around x f(x+Dx) = f(x)+J*Dx where Dx = (J^T*J)^-1*J^T*(b-f(x))
// 	float xkc = p(0);
// 	float ykc = p(1);
// 	float zkc = p(2);
// 	float qw = q(0);
// 	float qx = q(1);
// 	float qy = q(2);
// 	float qz = q(3);
// 	float r11 = 1.0-2.0*(qy*qy+qz*qz);
// 	float r12 = 2.0*(qx*qy-qz*qw);
// 	float r13 = 2.0*(qx*qz+qy*qw);
// 	float r21 = 2.0*(qx*qy+qz*qw);
// 	float r22 = 1.0-2.0*(qx*qx+qz*qz);
// 	float r23 = 2.0*(qy*qz-qx*qw);
// 	float r31 = 2.0*(qw*qz-qy*qw);
// 	float r32 = 2.0*(qy*qz+qx*qw);
// 	float r33 = 1.0-2.0*(qx*qx+qy*qy);
// 	float xic = zic*(uic-cx)/fx;
// 	float yic = zic*(vic-cy)/fy;
// 	float xik = r11*(xic-xkc)+r21*(yic-ykc)+r31*(zic-zkc);
// 	float yik = r12*(xic-xkc)+r22*(yic-ykc)+r32*(zic-zkc);
// 	float zik = r13*(xic-xkc)+r23*(yic-ykc)+r33*(zic-zkc);
//
// 	//partials of uik
// 	float dukduc = fx*(zik*r11*zic/fx-xik*r13*zic/fx)/(zik*zik);
// 	float dukdvc = fx*(zik*r21*zic/fy-xik*r23*zic/fy)/(zik*zik);
// 	float dukdzc = fx*(zik*(r11*(uic-cx)/fx+r21*(vic-cy)/fy+r31)-xik*(r13*(uic-cx)/fx+r23*(vic-cy)/fy+r33))/(zik*zik);
// 	float dukdqw = fx*(zik*((2.0*qz*(zic*(vic-cy)/fy-ykc))+2.0*(qz-qy)*(zic-zkc))-xik*(2.0*qy*(zic*(uic-cx)/fx-xkc)-2.0*qx*(zic*(vic-cy)/fy-ykc)))/(zik*zik);
// 	float dukdqx = fx*(zik*(2.0*qy*(zic*(vic-cy)/fy-ykc))-xik*(2.0*qz*(zic*(uic-cx)/fx-xkc)-2.0*qw*(zic*(vic-cy)/fy-ykc)-4.0*qx*(zic-zkc)))/(zik*zik);
// 	float dukdqy = fx*(zik*(-4.0*qy*(zic*(uic-cx)/fx-xkc)+2.0*qx*(zic*(vic-cy)/fy-ykc)-2.0*qw*(zic-zkc))-xik*(2.0*qw*(zic*(uic-cx)/fx-xkc)+2.0*qz*(zic*(vic-cy)/fy-ykc)-4.0*qy*(zic-zkc)))/(zik*zik);
// 	float dukdqz = fx*(zik*(-4.0*qz*(zic*(uic-cx)/fx-xkc)+2.0*qw*(zic*(vic-cy)/fy-ykc)+2.0*qw*(zic-zkc))-xik*(2.0*qx*(zic*(uic-cx)/fx-xkc)+2.0*qy*(zic*(vic-cy)/fy-ykc)))/(zik*zik);
// 	float dukdxkc = fx*(-r11*zik+r13*xik)/(zik*zik);
// 	float dukdykc = fx*(-r21*zik+r23*xik)/(zik*zik);
// 	float dukdzkc = fx*(-r31*zik+r33*xik)/(zik*zik);
//
// 	//partials of vik
// 	float dvkduc = fy*(zik*r12*zic/fx-yik*r13*zic/fx)/(zik*zik);
// 	float dvkdvc = fy*(zik*r22*zic/fy-yik*r23*zic/fy)/(zik*zik);
// 	float dvkdzc = fy*(zik*(r12*(uic-cx)/fx+r22*(vic-cy)/fy+r32)-yik*(r13*(uic-cx)/fx+r23*(vic-cy)/fy+r33))/(zik*zik);
// 	float dvkdqw = fy*(zik*((-2.0*qz*(zic*(uic-cx)/fx-xkc))+2.0*qx*(zic-zkc))-yik*(2.0*qy*(zic*(uic-cx)/fx-xkc)-2.0*qx*(zic*(vic-cy)/fy-ykc)))/(zik*zik);
// 	float dvkdqx = fy*(zik*(2.0*qy*(zic*(uic-cx)/fx-xkc)-4.0*qx*(zic*(vic-cy)/fy-ykc)+2.0*qw*(zic-zkc))-yik*(2.0*qz*(zic*(uic-cx)/fx-xkc)-2.0*qw*(zic*(vic-cy)/fy-ykc)-4.0*qx*(zic-zkc)))/(zik*zik);
// 	float dvkdqy = fy*(zik*(2.0*qx*(zic*(uic-cx)/fx-xkc)+2.0*qz*(zic-zkc))-yik*(2.0*qw*(zic*(uic-cx)/fx-xkc)+2.0*qz*(zic*(vic-cy)/fy-ykc)-4.0*qy*(zic-zkc)))/(zik*zik);
// 	float dvkdqz = fy*(zik*(-2.0*qw*(zic*(uic-cx)/fx-xkc)-4.0*qz*(zic*(vic-cy)/fy-ykc)+2.0*qy*(zic-zkc))-yik*(2.0*qx*(zic*(uic-cx)/fx-xkc)+2.0*qy*(zic*(vic-cy)/fy-ykc)))/(zik*zik);
// 	float dvkdxkc = fy*(-r12*zik+r13*yik)/(zik*zik);
// 	float dvkdykc = fy*(-r22*zik+r23*yik)/(zik*zik);
// 	float dvkdzkc = fy*(-r32*zik+r33*yik)/(zik*zik);
//
// 	Eigen::Matrix<float,2,10> Ji;
// 	Ji << dukduc,dukdvc,dukdzc,dukdqw,dukdqx,dukdqy,dukdqz,dukdxkc,dukdykc,dukdzkc,
// 	     dvkduc,dvkdvc,dvkdzc,dvkdqw,dvkdqx,dvkdqy,dvkdqz,dvkdxkc,dvkdykc,dvkdzkc;
// 	return Ji;
// }
