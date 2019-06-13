#include <depth_estimator_icl_ext.h>

DepthEstimatorICLExt::DepthEstimatorICLExt()
{
  uvInt = Eigen::Vector2f::Zero();
  yysum = 0.0;
  yusum = 0.0;
  firstzk = true;
  dkKnown = false;
  numSaved = 0;
}
void DepthEstimatorICLExt::initialize(Eigen::Vector3f uInit, float zminInit, float zmaxInit, float zInit, float tauInit, ros::Time t)
{
    uk = uInit;
    zmin = zminInit;
    zmax = zmaxInit;
    dkHat = zInit;
    dcHat = zInit;
    dkcHat = 0.0;
    tau = tauInit;
    uDotEstimator.initialize();
  	uDotEstimator.update(uk,t);
}

Eigen::Vector3f DepthEstimatorICLExt::update(Eigen::Vector3f ucMeas, Eigen::Vector3f ukc, Eigen::Matrix3f Rkc, Eigen::Vector3f v, Eigen::Vector3f w, Eigen::Vector3f pkc, ros::Time t, float dt)
{
  // std::cout << "\n hi4 \n";

  float kzk = 10.0;

  Eigen::Matrix<float,6,1> xHat = uDotEstimator.update(ucMeas,t);
  Eigen::Vector3f uc = xHat.segment(0,3);
  Eigen::Vector3f ucDot = xHat.segment(3,3);
  Eigen::RowVector3f ucT = uc.transpose();
  Eigen::RowVector3f ukcT = ukc.transpose();

  // std::cout << "\n hi5 \n";

  Eigen::Matrix<float,3,2> Yc;
  Eigen::Matrix<float,2,3> YcT;
  Yc.block(0,0,3,1) = uc;
  Yc.block(0,1,3,1) = -ukc;
  YcT.block(0,0,1,3) = ucT;
  YcT.block(1,0,1,3) = -ukcT;
  Eigen::Matrix2f YcYc = YcT*Yc;
  float YcYcdet = YcYc(0,0)*YcYc(1,1) - YcYc(0,1)*YcYc(1,0);
  Eigen::Matrix2f YcYcI;
  YcYcI(0,0) = YcYc(1,1)/YcYcdet;
  YcYcI(0,1) = -YcYc(0,1)/YcYcdet;
  YcYcI(1,0) = -YcYc(1,0)/YcYcdet;
  YcYcI(1,1) = YcYc(0,0)/YcYcdet;

  // std::cout << "\n hi6 \n";

  Eigen::Vector2f zeta = YcYcI*YcT*Rkc*uk;
  float dcDot = -ucT*v;
  float dkcDot = -ukcT*v;
  Eigen::Vector2f uv(dcDot,dkcDot);

  // std::cout << "\n hi7 \n";

  Eigen::Vector3f xi = ucDot + getss(w)*uc;
  Eigen::RowVector3f xiT = xi.transpose();
  Eigen::Vector3f rho = (uc*ucT - Eigen::Matrix3f::Identity())*v;
  float xixi = xiT*xi;
  float xirho = xiT*rho;
  float kxixiTilde = 1.5*kzk*(xirho - xixi*dcHat);

  // std::cout << "\n hi8 \n";

  // std::cout << "\n vx " << v(0) << " vy " << v(1) << " vz " << v(2) << std::endl;
  // std::cout << "\n tx " << tkc(0) << " ty " << tkc(1) << " tz " << tkc(2) << std::endl;
  // std::cout << "\n ux " << ukc(0) << " uy " << ukc(1) << " uz " << ukc(2) << std::endl;

  dcHat += ((dcDot+kxixiTilde)*dt);
  dkcHat += (dkcDot*dt);

  zetaBuff.push_back(zeta);
  uvBuff.push_back(uv);
  tBuff.push_back(t);
  dtBuff.push_back(dt);
  uvInt += (uv*dt);

  // std::cout << "\n hi9 \n";

  if ((1.0-fabsf(ucT*ukc) < 0.1))
  {
    zetaBuff.clear();
    uvBuff.clear();
    tBuff.clear();
    dtBuff.clear();
    uvInt = Eigen::Vector2f::Zero();
  }

  if (v.norm() < 0.05)
  {
    zetaBuff.clear();
    uvBuff.clear();
    tBuff.clear();
    dtBuff.clear();
    uvInt = Eigen::Vector2f::Zero();
  }

  if (pkc.norm() < 0.03)
  {
    zetaBuff.clear();
    uvBuff.clear();
    tBuff.clear();
    dtBuff.clear();
    uvInt = Eigen::Vector2f::Zero();
  }

  // std::cout << "\n hi10 \n";

  if (tBuff.size() > 3)
  {


    // std::cout << "\n tBuff.size() " << tBuff.size() << " uvBuff.size() " << uvBuff.size() << " zetaBuff.size() " << zetaBuff.size() << " dtBuff.size() " << dtBuff.size() << std::endl;

    bool timeGood = false;
    while (!timeGood)
    {
      if (tBuff.size() > 2)
      {
        if ((tBuff.at(tBuff.size()-1) - tBuff.at(1)).toSec() > tau)
        {
          uvInt -= (uvBuff.at(0)*dtBuff.at(0));
          zetaBuff.pop_front();
          uvBuff.pop_front();
          tBuff.pop_front();
          dtBuff.pop_front();
        }
        else
        {
          timeGood = true;
        }
      }
      else
      {
        timeGood = true;
      }
    }

    // std::cout << "\n hi11 \n";

    Eigen::Vector2f Y = zetaBuff.at(zetaBuff.size()-1) - zetaBuff.at(0);
    Eigen::Vector2f U = uvInt;

    float Yx = Y(0);
    float Ux = U(0);

    float Yy = Y(1);
    float Uy = U(1);

    // std::cout << "\n hi12 \n";

    // std::cout << "\n mzetaBuffInt \n" << mzetaBuffInt << std::endl;
    // std::cout << "\n psizetaBuffInt \n" << psizetaBuffInt << std::endl;

    // std::cout << "\n (Ux/Yx) " << (Ux/Yx) << std::endl;
    // std::cout << "\n Yx " << Yx << std::endl;
    // std::cout << "\n Ux " << Ux << std::endl;
    //
    // std::cout << "\n (Uy/Yy)  " << (Uy/Yy) << std::endl;
    // std::cout << "\n Yy " << Yy << std::endl;
    // std::cout << "\n Uy " << Uy << std::endl;

    //check which estimates are good
    bool measgood = (U.norm() > 0.1) && (Y.norm() > 0.1);
    // bool xGood = (fabsf(Ux) > 0.1) && (fabsf(Yx) > 0.1);
    // bool yGood = (fabsf(Uy) > 0.1) && (fabsf(Yy) > 0.1);

    bool dirGoodzBad = false;

    // std::cout << "\n hi13 \n";

    //check the ys
    if (measgood && (numSaved < 50))
    {
      float yy = Yx*Yx + Yy*Yy;
      float yu = Yx*Ux+Yy*Uy;
      float dk = yu/yy;
      if ((dk > zmin) && (dk < zmax))
      {
        yysum += yy;
        yusum += yu;
        numSaved++;
      }
      else
      {
        dirGoodzBad = true;
      }
    }

    if (dirGoodzBad)
    {
      zetaBuff.clear();
      uvBuff.clear();
      tBuff.clear();
      dtBuff.clear();
      uvInt = Eigen::Vector2f::Zero();
    }
  }

  // std::cout << "\n hi14 \n";

  if (yysum > 0.0001)
  {
    dkKnown = true;
    float dkMed = yusum/yysum;

    // std::cout << "\n zkMed " << zkMed << std::endl;
    // std::cout << "\n zkHat " << zkHat << std::endl;

    if (dkMed < zmin)
    {
      dkMed = zmin;
    }

    if (dkMed > zmax)
    {
      dkMed = zmax;
    }



    Eigen::Vector2f ddTil = (dkMed*YcT*Rkc*uk - YcYc*Eigen::Vector2f(dcHat,dkcHat));

    // std::cout << "\n dkMed*YcT*Rkc*uk \n" << dkMed*YcT*Rkc*uk << std::endl;
    // std::cout << "\n YcYc*Eigen::Vector2f(dcHat,dkcHat) \n" << YcYc*Eigen::Vector2f(dcHat,dkcHat) << std::endl;
    // std::cout << "\n ddTil0 " << ddTil(0) << " ddTil1 " << ddTil(1) << std::endl;

    dcHat += (kzk*ddTil(0)*dt);
    dkcHat += (kzk*ddTil(1)*dt);
    dkHat += (kzk*(dkMed-dkHat)*dt);

    // dcHat += (kzk*(zeta(0)*dkHat-dcHat)*dt);
    // dkcHat += (kzk*(zeta(1)*dkHat-dkcHat)*dt);
    // dkHat += (kzk*(yusum-yysum*dkHat)*dt);
  }

  // std::cout << "\n hi15 \n";
  if (dcHat > zmax)
  {
    dcHat = zmax;
  }

  if (dcHat < zmin)
  {
    dcHat = zmin;
  }

  if (dkHat > zmax)
  {
    dkHat = zmax;
  }

  if (dkHat < zmin)
  {
    dkHat = zmin;
  }

  if (dkcHat < 0.01)
  {
    dkcHat = 0.01;
  }

  return Eigen::Vector3f(dkHat,dcHat,dkcHat);
}
