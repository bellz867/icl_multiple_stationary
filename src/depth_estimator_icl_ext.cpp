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
  timeConverge = 0.0;
}
void DepthEstimatorICLExt::initialize(Eigen::Vector3f uInit, float zminInit, float zmaxInit, float zInit,
                                      float tauInit, ros::Time t, float fxInit, float fyInit, float cxInit, float cyInit)
{
    uk = uInit;
    uk /= uk.norm();
    up = uk;
    zmin = zminInit;
    zmax = zmaxInit;
    dkHat = zInit;
    dcHat = zInit;
    dkcHat = 0.0;
    tau = tauInit;
    uDotEstimator.initialize(3);
  	// uDotEstimator.update(uk,t);
    psiDotEstimator.initialize(2);
    fx = fxInit;
    fy = fyInit;
    cx = cxInit;
    cy = cyInit;
    startTime = t;
}

Eigen::Vector3f DepthEstimatorICLExt::current()
{
  // Eigen::Vector3f uc = uDotEstimator.xHat.segment(0,3);
  Eigen::Vector3f uc = up;
  if (uc.norm()>0.001)
  {
    uc /= uc.norm();
    uc /= uc(2);
    return uc;
  }
  else
  {
    return Eigen::Vector3f::Zero();
  }
}

Eigen::Vector3f DepthEstimatorICLExt::predict(Eigen::Vector3f v, Eigen::Vector3f w, float dt, Eigen::Vector3f pkc, Eigen::Vector4f qkc)
{
  Eigen::Vector3f pcProj = pkc + rotatevec(uk*dkHat,qkc);
  Eigen::Vector3f mcProj = pcProj/pcProj(2);
  Eigen::Vector2f cPtProj(mcProj(0)*fx+cx,mcProj(1)*fy+cy);

  Eigen::Vector3f mp = current();
  Eigen::Vector2f pPt(mp(0)*fx+cx,mp(1)*fy+cy);

  Eigen::Vector3f ucEst = up;
  if (ucEst.norm()>0.001)
  {
    ucEst /= ucEst.norm();
  }
  else
  {
    return mcProj;
  }

  Eigen::RowVector3f ucEstT = ucEst.transpose();
  Eigen::Vector3f ucEstDot = -getss(w)*ucEst + (1.0/dcHat)*(ucEst*ucEstT - Eigen::Matrix3f::Identity())*v;
  // Eigen::Vector3f ucDotEst = uDotEstimator.xHat.segment(3,3);
  // std::cout << "\n dcHat " << dcHat << std::endl;
  // std::cout << "\n ucDot " << ucEstDot << std::endl;
  // std::cout << "\n ucDotf " << ucDotEst << std::endl;
  // std::cout << "\n ucb " << ucEst << std::endl;
  ucEst += (ucEstDot*dt);
  ucEst /= ucEst.norm();
  // std::cout << "\n uca " << ucEst << std::endl;
  Eigen::Vector3f mcEst = ucEst /= ucEst(2);
  Eigen::Vector2f cPtEst(mcEst(0)*fx+cx,mcEst(1)*fy+cy);

  float sigEst = 5;
  float sigEst2 = sigEst*sigEst;
  float chi2 = 6.63; //chi^2 for 99%
  Eigen::Vector2f cPtD = cPtProj - cPtEst;
  float chiTestVal = (cPtD(0)*cPtD(0) + cPtD(1)*cPtD(1))/sigEst2;
  float sigProj = sigEst+chiTestVal;
  float sigProj2 = sigProj*sigProj;
  float sig2Sum = sigEst2+sigProj2;
  // float chi2 = 3.84; //chi^2 for 95%

  // float cPtAlpha = 1.0/(2.0 + chiTestVal);
  Eigen::Vector2f cPtComb((sigProj2/sig2Sum)*cPtEst(0)+(sigEst2/sig2Sum)*cPtProj(0),(sigProj2/sig2Sum)*cPtEst(1)+(sigEst2/sig2Sum)*cPtProj(1));

  std::cout << std::endl << "dkKnown " << int(dkKnown) << ", chiTestVal " << chiTestVal;
  std::cout << ", cPtProjx " << cPtProj(0) << ", cPtProjy " << cPtProj(1);
  std::cout << ", cPtEstx " << cPtEst(0) << ", cPtEsty " << cPtEst(1);
  std::cout << ", cPtCombx " << cPtComb(0) << ", cPtComby " << cPtComb(1);
  std::cout << ", pPtx " << pPt(0) << ", pPty " << pPt(1);
  std::cout << std::endl;

  Eigen::Vector3f mcComb((cPtComb(0)-cx)/fx,(cPtComb(1)-cy)/fy,1.0);

  // if (chiTestVal > chi2)
  // {
  //   return mcEst;
  // }

  return mcComb;
}

Eigen::Vector3f DepthEstimatorICLExt::update(Eigen::Vector3f ucMeas, Eigen::Vector3f ukc, Eigen::Matrix3f Rkc, Eigen::Vector3f v, Eigen::Vector3f w, ros::Time t, float dt, Eigen::Vector3f pkc, Eigen::Vector4f qkc)
{
  // std::cout << "\n hi4 \n";

  float kxi = 250.0;
  float kX = 25.0;

  Eigen::Matrix<float,6,1> xHat = uDotEstimator.update(ucMeas,t);
  Eigen::Vector3f uc = xHat.segment(0,3);

  if (uc.norm()>0.001)
  {
    uc /= uc.norm();
  }

  Eigen::Vector3f mc = uc;
  if (fabsf(mc(2))>0.001)
  {
    mc /= mc(2);
  }
  up = uc;

  Eigen::Vector2f cPt(fx*mc(0)+cx,fy*mc(1)+cy);

  Eigen::Vector3f ucDot = xHat.segment(3,3);
  Eigen::RowVector3f ucT = uc.transpose();
  Eigen::RowVector3f ukcT = ukc.transpose();

  // std::cout << "\n hi5 \n";

  float ucukc = ucT*ukc;
  float YcYcdet = 1.0 - ucukc*ucukc;
  Eigen::Matrix2f YcYc = Eigen::Matrix2f::Identity();
  YcYc(0,1) = -ucukc;
  YcYc(1,0) = -ucukc;
  Eigen::Matrix2f YcYcI = Eigen::Matrix2f::Identity();;
  float YcYcdetI = 1.0/YcYcdet;
  YcYcI(0,1) = ucukc;
  YcYcI(1,0) = ucukc;
  YcYcI /= YcYcdet;
  // Eigen::Matrix<float,3,2> Yc;
  Eigen::Matrix<float,2,3> YcT;
  // Yc.block(0,0,3,1) = uc;
  // Yc.block(0,1,3,1) = -ukc;
  YcT.block(0,0,1,3) = ucT;
  YcT.block(1,0,1,3) = -ukcT;
  // Eigen::Matrix2f YcYc = YcT*Yc;
  // float YcYcdet = YcYc(0,0)*YcYc(1,1) - YcYc(0,1)*YcYc(1,0);
  // Eigen::Matrix2f YcYcI;
  // YcYcI(0,0) = YcYc(1,1)/YcYcdet;
  // YcYcI(0,1) = -YcYc(0,1)/YcYcdet;
  // YcYcI(1,0) = -YcYc(1,0)/YcYcdet;
  // YcYcI(1,1) = YcYc(0,0)/YcYcdet;

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

  Eigen::Matrix<float,3,2> Yk;
  Eigen::Matrix<float,2,3> YkT;
  Yk.block(0,0,3,1) = ukc;
  Yk.block(0,1,3,1) = Rkc*uk;
  YkT = Yk.transpose();
  Eigen::Matrix2f YkYk = YkT*Yk;
  Eigen::Vector2f Ykuc = YkT*uc;
  Eigen::Vector2f kxpxpTilde = 0.1*kxi*(Ykuc*dcHat - YkYk*Eigen::Vector2f(dkcHat,dkHat));

  // float ucukc = ucT*ukc;
  // float ucRuk = ucT*Rkc*ukc;
  // Eigen::RowVector2f psiu(ucukc,ucRuk);
  // Eigen::Matrix<float,3,2> xipsiu = xi*psiu;
  // Eigen::Matrix<float,2,3> xipsiuT = xipsiu.transpose();
  // Eigen::Matrix2f xpxp = xipsiuT*xipsiu;
  // Eigen::Vector2f xprho = xipsiuT*rho;
  // Eigen::Vector2f kxpxpTilde = kxi*(xprho - xpxp*Eigen::Vector2f(dkcHat,dkHat));


  // std::cout << "\n hi8 \n";

  // std::cout << "\n vx " << v(0) << " vy " << v(1) << " vz " << v(2) << std::endl;
  // std::cout << "\n tx " << tkc(0) << " ty " << tkc(1) << " tz " << tkc(2) << std::endl;
  // std::cout << "\n ucx " << uc(0) << " ucy " << uc(1) << " ucz " << uc(2) << std::endl;
  // std::cout << "\n ucDotx " << ucDot(0) << " ucDoty " << ucDot(1) << " ucDotz " << ucDot(2) << std::endl;
  // std::cout << "\n ukcx " << ukc(0) << " ukcy " << ukc(1) << " ukcz " << ukc(2) << std::endl;
  // std::cout << "\n px " << pkc(0) << " py " << pkc(1) << " pz " << pkc(2) << std::endl;
  // std::cout << "\n px " << pkc(0) << " py " << pkc(1) << " pz " << pkc(2) << std::endl;
  // std::cout << "\n px " << pkc(0) << " py " << pkc(1) << " pz " << pkc(2) << std::endl;
  // std::cout << "\n px " << pkc(0) << " py " << pkc(1) << " pz " << pkc(2) << std::endl;


  if (pkc.norm() < 0.05)
  {
    dcHat += (dcDot*dt);
    dkcHat += (dkcDot*dt);
  }
  else
  {
    dcHat += ((dcDot+kxixiTilde)*dt);
    // dkcHat += (dkcDot*dt);
    dkcHat += ((dkcDot+kxpxpTilde(0))*dt);
    dkHat += (kxpxpTilde(1)*dt);
  }

  // dkcHat += ((dkcDot)*dt);
  // dkHat += (kxpxpTilde(1)*dt);

  psiBuff.push_back(psi);
  psiDotBuff.push_back(psiDot);
  uvBuff.push_back(uv);
  tBuff.push_back(t);
  dtBuff.push_back(dt);
  uvInt += (uv*dt);
  psiDotInt += (psiDot*dt);

  // std::cout << "\n hi9 \n";
  float lambdaa = 0.5;
  float lambdat = 0.0001;
  float dmin = 0.1*zmin;
  float dmax = zmax;
  float timeConvergeMin = -1.0/(lambdaa*kX)*log(dmin/dmax);

  if ((1.0-fabsf(ucT*ukc) < lambdaa))
  {
    psiBuff.clear();
    psiDotBuff.clear();
    uvBuff.clear();
    tBuff.clear();
    dtBuff.clear();
    uvInt = Eigen::Vector2f::Zero();
    psiDotInt = Eigen::Vector2f::Zero();
    // uDotEstimator.initialize(3);
    // psiDotEstimator.initialize(2);
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
    // uDotEstimator.initialize(3);
    // psiDotEstimator.initialize(2);
    numThrown++;
    std::cout << "\n v clear\n";
  }

  if (pkc.norm() < 0.05)
  {
    psiBuff.clear();
    psiDotBuff.clear();
    uvBuff.clear();
    tBuff.clear();
    dtBuff.clear();
    uvInt = Eigen::Vector2f::Zero();
    psiDotInt = Eigen::Vector2f::Zero();
    // uDotEstimator.initialize(3);
    // psiDotEstimator.initialize(2);
    // numThrown++;
    std::cout << "\n pkc clear\n";
  }

  std::cout << "\n" << "N " << numSaved << ", dc  " << dcHat << ", dkc  " << dkcHat << ", dk  " << dkHat << ", dkcdd " << (ukcT*uc*dcHat-ukcT*Rkc*uk*dkHat);

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

    // Eigen::Vector2f Y = psiDotInt;
    Eigen::Vector2f Y = (psiBuff.at(psiBuff.size()-1)-psiBuff.at(0));
    Eigen::Vector2f U = uvInt;

    std::cout << ", psiDotInt(0)  " << psiDotInt(0) << ", psiDotInt(1)  " << psiDotInt(1) << ", Y(0)  " << Y(0) << ", Y(1)  " << Y(1) << ", U(0)  " << U(0) << ", U(1)  " << U(1);

    float Yx = Y(0);
    float Ux = U(0);

    float Yy = Y(1);
    float Uy = U(1);

    float Yx2 = psiDotInt(0);
    float Yy2 = psiDotInt(1);

    // std::cout << "\n hi12 \n";

    // std::cout << "\n mzetaBuffInt \n" << mzetaBuffInt << std::endl;
    // std::cout << "\n psizetaBuffInt \n" << psizetaBuffInt << std::endl;

    // std::cout << "\n (Ux/Yx) " << (Ux/Yx) << std::endl;
    // std::cout << "\n Yx " << Yx << std::endl;
    // std::cout << "\n Ux " << Ux << std::endl;
    // float yy = Yx*Yx + Yy*Yy;
    // float yy2 = Yx2*Yx2 + Yy2*Yy2;
    // float yu = Yx*Ux+Yy*Uy;
    // float yu2 = Yx2*Ux+Yy2*Uy;
    // float dk = yu/yy;
    // float dk2 = yu2/yy2;
    // float dkMed = yusum/yysum;;
    float yy = Yy*Yy;
    float yy2 = Yy2*Yy2;
    float yu = Yy*Uy;
    float yu2 = Yy2*Uy;
    float dk = yu/yy;
    float dk2 = yu2/yy2;
    float dkMed = yusum/yysum;

    Eigen::Vector3f Rkcuk = rotatevec(uk,qkc);
    float ucRkcuk = ucT*Rkcuk;
    float YtYtdet = 1.0 - ucRkcuk*ucRkcuk;
    Eigen::Matrix2f YtYtI = Eigen::Matrix2f::Identity();
    YtYtI(0,1) = ucRkcuk;
    YtYtI(1,0) = ucRkcuk;
    YtYtI /= YtYtdet;
    Eigen::Matrix<float,2,3> YtT;
    YtT.block(0,0,1,3) = ucT;
    YtT.block(1,0,1,3) = -Rkcuk.transpose();
    Eigen::Vector2f dcdkt = YtYtI*YtT*pkc;

    std::cout << ", dct " << dcdkt(0) <<  ", dkt  " << dcdkt(1);
    std::cout << ", dkMed " << dkMed <<  ", dk  " << dk << ", dk2 " << dk2 << ", |Y| " << Y.norm() << ", |U| ";
    std::cout << U.norm() << ", psi(0)*dk " << psi(0)*dk << ", psi(1)*dk " << psi(1)*dk;
    std::cout << ", psi(0)*dkMed " << psi(0)*dkMed << ", psi(1)*dkMed " << psi(1)*dkMed << std::endl;
    // std::cout << "\n yu " << yu << std::endl;
    // std::cout << "\n Y.norm() " << Y.norm() << std::endl;
    // std::cout << "\n U.norm() " << U.norm() << std::endl;
    // std::cout << "\n Dt " << (tBuff.at(tBuff.size()-1) - tBuff.at(0)).toSec() << std::endl;

    //check which estimates are good
    bool measgood = (fabsf(Uy) > 0.2) && (fabsf(Yy) > 0.1);
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

    if (measgood)
    {
      //chi^2 test for reprojection error using dk
      // assume pixel standard deviation of 2 implying variance of 4
      float cPtSig = 20;
      float cPtSig2 = cPtSig*cPtSig;
      Eigen::Vector3f pcProj = pkc + rotatevec(uk*dk,qkc);
      Eigen::Vector3f mcProj = pcProj/pcProj(2);
      Eigen::Vector2f cPtProj(mcProj(0)*fx+cx,mcProj(1)*fy+cy);
      // float chi2 = 3.84; //chi^2 for 95%
      float chi2 = 6.63; //chi^2 for 99%
      Eigen::Vector2f cPtD = cPtProj - cPt;
      float chiTestVal = (cPtD(0)*cPtD(0) + cPtD(1)*cPtD(1))/cPtSig2;

      std::cout << std::endl;
      std::cout << "cPtProjx " << cPtProj(0) << ", cPtProjy " << cPtProj(1);
      std::cout << ", cPtx " << cPt(0) << ", cPty " << cPt(1) << ", chiTestVal " << chiTestVal;
      std::cout << std::endl;

      //if the value is outside the acceptable then reject
      if (chiTestVal > chi2)
      {
        //if the test failed with the first dk try the second
        pcProj = pkc + rotatevec(uk*dk2,qkc);
        mcProj = pcProj/pcProj(2);
        cPtProj = Eigen::Vector2f(mcProj(0)*fx+cx,mcProj(1)*fy+cy);
        // float chi2 = 3.84; //chi^2 for 95%
        // float chi2 = 6.63; //chi^2 for 99%
        cPtD = cPtProj - cPt;
        chiTestVal = (cPtD(0)*cPtD(0) + cPtD(1)*cPtD(1))/cPtSig2;

        std::cout << std::endl;
        std::cout << "cPt2Projx " << cPtProj(0) << ", cPtProj2y " << cPtProj(1);
        std::cout << ", cPtx " << cPt(0) << ", cPty " << cPt(1) << ", chiTestVal2 " << chiTestVal;
        std::cout << std::endl;

        if (chiTestVal > chi2)
        {
          measgood = false;
        }
        else
        {
          dk = dk2;
        }
      }
    }

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
      // uDotEstimator.initialize(3);
      // psiDotEstimator.initialize(2);
    }
  }
  else
  {
    // std::cout << "\n";
  }

  // std::cout << "\n hi14 \n";

  if (yysum > lambdat)
  {
    timeConverge += dt;
    if (timeConverge > timeConvergeMin)
    {
      dkKnown = true;
    }

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
