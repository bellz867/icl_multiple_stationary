#include <depth_estimator_icl_ext.h>

DepthEstimatorICLExt::DepthEstimatorICLExt()
{
  uvInt = Eigen::Vector2f::Zero();
  psiDotInt = Eigen::Vector2f::Zero();
  yysum = 0.0;
  yusum = 0.0;
  firstzk = true;
  dkKnown = false;
  numSaved = 0;
  numThrown = 0;
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
    uDotEstimator.initialize(3);
  	uDotEstimator.update(uk,t);
    psiDotEstimator.initialize(2);
}

Eigen::Vector3f DepthEstimatorICLExt::update(Eigen::Vector3f ucMeas, Eigen::Vector3f ukc, Eigen::Matrix3f Rkc, Eigen::Vector3f v, Eigen::Vector3f w, Eigen::Vector3f pkc, ros::Time t, float dt)
{
  // std::cout << "\n hi4 \n";

  float kxi = 75.0;
  float kX = 10.0;

  Eigen::Matrix<float,6,1> xHat = uDotEstimator.update(ucMeas,t);
  Eigen::Vector3f uc = xHat.segment(0,3);

  if (uc.norm()>0.001)
  {
    uc /= uc.norm();
  }

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

  Eigen::Vector2f psiMeas = YcYcI*YcT*Rkc*uk;
  Eigen::Matrix<float,4,1> Xpsi = psiDotEstimator.update(psiMeas,t);
  Eigen::Vector2f psi = Xpsi.segment(0,2);
  Eigen::Vector2f psiDot = Xpsi.segment(2,2);

  float dcDot = -ucT*v;
  float dkcDot = -ukcT*v;
  Eigen::Vector2f uv(dcDot,dkcDot);

  // std::cout << "\n hi7 \n";

  // Eigen::Vector3f xi = ucDot + getss(w)*uc;
  // Eigen::RowVector3f xiT = xi.transpose();
  // Eigen::Vector3f rho = (uc*ucT - Eigen::Matrix3f::Identity())*v;
  // float xixi = xiT*xi;
  // float xirho = xiT*rho;
  // float kxixiTilde = 1.5*kzk*(xirho - xixi*dcHat);
  Eigen::Vector3f xi = ucDot + getss(w)*uc;
  Eigen::RowVector3f xiT = xi.transpose();
  Eigen::Vector3f rho = (uc*ucT - Eigen::Matrix3f::Identity())*v;
  float xixi = xiT*xi;
  float xirho = xiT*rho;
  float kxixiTilde = kxi*(xirho - xixi*dcHat);
  float ucukc = ucT*ukc;
  float ucRuk = ucT*Rkc*ukc;
  Eigen::RowVector2f psiu(ucukc,ucRuk);
  Eigen::Matrix<float,3,2> xipsiu = xi*psiu;
  Eigen::Matrix<float,2,3> xipsiuT = xipsiu.transpose();
  Eigen::Matrix2f xpxp = xipsiuT*xipsiu;
  Eigen::Vector2f xprho = xipsiuT*rho;
  Eigen::Vector2f kxpxpTilde = kxi*(xprho - xpxp*Eigen::Vector2f(dkcHat,dkHat));


  // std::cout << "\n hi8 \n";

  // std::cout << "\n vx " << v(0) << " vy " << v(1) << " vz " << v(2) << std::endl;
  // std::cout << "\n tx " << tkc(0) << " ty " << tkc(1) << " tz " << tkc(2) << std::endl;
  // std::cout << "\n ux " << ukc(0) << " uy " << ukc(1) << " uz " << ukc(2) << std::endl;
  // std::cout << "\n px " << pkc(0) << " py " << pkc(1) << " pz " << pkc(2) << std::endl;


  dcHat += ((dcDot+kxixiTilde)*dt);
  // dkcHat += ((dkcDot+kxpxpTilde(0))*dt);
  // dkHat += (kxpxpTilde(1)*dt);
  dkcHat += ((dkcDot)*dt);
  // dkHat += (kxpxpTilde(1)*dt);

  psiBuff.push_back(psi);
  psiDotBuff.push_back(psiDot);
  uvBuff.push_back(uv);
  tBuff.push_back(t);
  dtBuff.push_back(dt);
  uvInt += (uv*dt);
  psiDotInt += (psiDot*dt);

  // std::cout << "\n hi9 \n";

  if ((1.0-fabsf(ucT*ukc) < 0.4))
  {
    psiBuff.clear();
    psiDotBuff.clear();
    uvBuff.clear();
    tBuff.clear();
    dtBuff.clear();
    uvInt = Eigen::Vector2f::Zero();
    psiDotInt = Eigen::Vector2f::Zero();
    numThrown++;
    std::cout << "\n eig clear\n";
  }

  if (v.norm() < 0.1)
  {
    psiBuff.clear();
    psiDotBuff.clear();
    uvBuff.clear();
    tBuff.clear();
    dtBuff.clear();
    uvInt = Eigen::Vector2f::Zero();
    psiDotInt = Eigen::Vector2f::Zero();
    numThrown++;
    std::cout << "\n v clear\n";
  }

  if (pkc.norm() < 0.1)
  {
    psiBuff.clear();
    psiDotBuff.clear();
    uvBuff.clear();
    tBuff.clear();
    dtBuff.clear();
    uvInt = Eigen::Vector2f::Zero();
    psiDotInt = Eigen::Vector2f::Zero();
    // numThrown++;
    std::cout << "\n pkc clear\n";
  }

  std::cout << "\n" << "dc  " << dcHat << ", dkc  " << dkcHat << ", dk  " << dkHat;

  // std::cout << "\n hi10 \n";

  if (tBuff.size() > 3)
  {
    // std::cout << "\n tBuff.size() " << tBuff.size() << " uvBuff.size() " << uvBuff.size() << " zetaBuff.size() " << zetaBuff.size() << " dtBuff.size() " << dtBuff.size() << std::endl;

    bool timeGood = false;

    float Dt = 0.0;
    while (!timeGood)
    {
      if (tBuff.size() > 2)
      {
        if ((tBuff.at(tBuff.size()-1) - tBuff.at(1)).toSec() > tau)
        {
          uvInt -= (uvBuff.at(0)*dtBuff.at(0));
          psiDotInt -= (psiDotBuff.at(0)*dtBuff.at(0));
          psiBuff.pop_front();
          psiDotBuff.pop_front();
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

    bool enoughTime = false;
    if ((tBuff.at(tBuff.size()-1) - tBuff.at(0)).toSec() > (0.25*tau))
    {
      enoughTime = true;
    }

    // std::cout << "\n hi11 \n";

    // Eigen::Vector2f Y = psiDotInt;
    Eigen::Vector2f Y = (psiBuff.at(psiBuff.size()-1)-psiBuff.at(0));
    Eigen::Vector2f U = uvInt;

    std::cout << ", psiDotInt(0)  " << psiDotInt(0) << ", psiDotInt(1)  " << psiDotInt(1) << ", Y(0)  " << Y(0) << ", Y(1)  " << Y(1);

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
    float yy = Yx*Yx + Yy*Yy;
    float yu = Yx*Ux+Yy*Uy;
    float dk = yu/yy;
    float dkMed = yusum/yysum;;

    std::cout << ", dkMed " << dkMed <<  ", dk  " << dk << ", |Y| " << Y.norm() << ", |U| ";
    std::cout << U.norm() << ", psi(0)*dk " << psi(0)*dk << ", psi(1)*dk " << psi(1)*dk;
    std::cout << ", psi(0)*dkMed " << psi(0)*dkMed << ", psi(1)*dkMed " << psi(1)*dkMed << std::endl;
    // std::cout << "\n yu " << yu << std::endl;
    // std::cout << "\n Y.norm() " << Y.norm() << std::endl;
    // std::cout << "\n U.norm() " << U.norm() << std::endl;
    // std::cout << "\n Dt " << (tBuff.at(tBuff.size()-1) - tBuff.at(0)).toSec() << std::endl;

    //check which estimates are good
    bool measgood = (U.norm() > 0.25) && (Y.norm() > 0.15);
    // bool disAgree = (fabsf(dcHat-zeta(0)*(yu/yy))/dcHat) < 0.3;
    // bool xGood = (fabsf(Ux) > 0.1) && (fabsf(Yx) > 0.1);
    // bool yGood = (fabsf(Uy) > 0.1) && (fabsf(Yy) > 0.1);

    // if (enoughTime && !measgood)
    // {
    //   zetaBuff.clear();
    //   uvBuff.clear();
    //   tBuff.clear();
    //   dtBuff.clear();
    //   uvInt = Eigen::Vector2f::Zero();
    //   numThrown++;
    // }


    bool dirGoodzBad = false;

    // std::cout << "\n hi13 \n";

    //check the ys
    // if (measgood && (numSaved < 50) && disAgree)
    if (measgood && (numSaved < 50))
    {
      bool saveNew = true;
      // if (numSaved > 0)
      // {
      //   if ((pkc-pkcLastSave).norm() < 0.1)
      //   {
      //     saveNew = false;
      //   }
      // }

      if ((dk > zmin) && (dk < zmax) && saveNew)
      {
        yysum += yy;
        yusum += yu;
        numSaved++;
        tLastSave = t;
        pkcLastSave = pkc;
      }
      else
      {
        dirGoodzBad = true;
      }
    }

    if (dirGoodzBad)
    {
      psiBuff.clear();
      psiDotBuff.clear();
      uvBuff.clear();
      tBuff.clear();
      dtBuff.clear();
      uvInt = Eigen::Vector2f::Zero();
      psiDotInt = Eigen::Vector2f::Zero();
    }
  }
  else
  {
    // std::cout << "\n";
  }

  // std::cout << "\n hi14 \n";

  if (yysum > 0.0001)
  {
    dkKnown = true;
    float dkMed = yusum/yysum;

    // std::cout << "\n dkMed " << dkMed << std::endl;
    // std::cout << "\n zkHat " << zkHat << std::endl;

    if (dkMed < zmin)
    {
      dkMed = zmin;
    }

    if (dkMed > zmax)
    {
      dkMed = zmax;
    }

    dkKnown = true;

    Eigen::Vector2f ddTil = (dkMed*YcT*Rkc*uk - YcYc*Eigen::Vector2f(dcHat,dkcHat));

    // std::cout << "\n dkMed*YcT*Rkc*uk \n" << dkMed*YcT*Rkc*uk << std::endl;
    // std::cout << "\n YcYc*Eigen::Vector2f(dcHat,dkcHat) \n" << YcYc*Eigen::Vector2f(dcHat,dkcHat) << std::endl;
    // std::cout << "\n ddTil0 " << ddTil(0) << " ddTil1 " << ddTil(1) << std::endl;

    dcHat += (kX*ddTil(0)*dt);
    dkcHat += (kX*ddTil(1)*dt);
    dkHat += (kX*(dkMed-dkHat)*dt);

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
