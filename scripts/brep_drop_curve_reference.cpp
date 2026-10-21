// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Public synthetic raw NURBS; no private CAD inputs or kernel rebuild.
#include "SmBSplineCurve.h"
#include "SmBSplineSurface.h"
#include "SmContext.h"
#include "SmPlane.h"
#include "SmSurface.h"
#include <nurbs.h>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char** argv) { try {
    if (argc < 5) return 2;
    std::string shape=argv[1],path=argv[2],mode=argv[3]; bool keep=std::stoi(argv[4])!=0;
    double scale=argc>5?std::stod(argv[5]):1.,offset=argc>6?std::stod(argv[6]):0.;
    SmContext context; std::cout<<std::setprecision(17);
    std::unique_ptr<SmSurface> surface;
    if(shape=="plane") surface.reset(new(&context) SmPlane(SmPoint3d(offset,offset,offset),SmVector3d(1,0,0),SmVector3d(0,1,0),SmVector2d(scale,scale),SmExtent2d(0,0,1,1),&context));
    else {
        bool cylinder=shape=="cylinder"||shape=="cone";
        size_t nu=cylinder?9:2,nv=2; std::vector<std::vector<gw_CPOINT>> rows(nu,std::vector<gw_CPOINT>(nv));std::vector<gw_CPOINT*> pointers;
        const double w=std::sqrt(2.)/2.;
        const std::array<std::array<double,2>,9> circle{{{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0}}};
        for(size_t i=0;i<nu;++i){for(size_t j=0;j<nv;++j){
            double x=cylinder?circle[i][0]:double(i),y=cylinder?circle[i][1]:double(j),z=cylinder?double(j):shape=="saddle"?double(i*j):0.;
            if(shape=="cone"){x*=double(j);y*=double(j);}
            if(shape=="point")x=y=z=0.;
            x=x*scale+offset;y=y*scale+offset;z=z*scale+offset;
            double weight=cylinder?(i%2?w:1.):-std::numeric_limits<double>::max();
            rows[i][j]={weight>0?x*weight:x,weight>0?y*weight:y,weight>0?z*weight:z,weight};
        }pointers.push_back(rows[i].data());}
        std::vector<double> u=cylinder?std::vector<double>{0,0,0,.25,.25,.5,.5,.75,.75,1,1,1}:std::vector<double>{0,0,1,1};
        double v[]{0,0,1,1};
        surface.reset(new(&context) SmBSplineSurface(ULONG(nu-1),1,cylinder?2:1,1,ULONG(u.size()-1),3,pointers.data(),u.data(),v));
    }
    surface->SetContext(&context);
    std::vector<SmPoint3d> points;
    if(path=="iso") points={{.2,.2,0},{.8,.2,0}};
    else if(path=="seam")points={{1,0,.2},{1,0,.8}};
    else if(path=="radial")points={{1.01,0,.2},{1.01,0,.8}};
    else if(path=="pole")points={{0,0,0},{0,0,0}};
    else if(path=="outside")points={{-.3,.2,0},{-.1,.8,0}};
    else if(path=="cross")points={{.1,.2,0},{1.2,.8,0}};
    else if(path=="curved")points={{.1,.2,.02},{.5,.45,.125},{.9,.7,.63}};
    else if(path=="quadratic")points={{.1,.2,0},{.3,.6,0},{.9,.7,0}};
    else if(path=="bulge")points={{.1,.2,0},{1.5,.6,0},{.9,.7,0}};
    else points={{.1,.2,0},{.9,.7,0}};
    std::vector<gw_CPOINT> cp;for(auto p:points)cp.push_back({p.x*scale+offset,p.y*scale+offset,p.z*scale+offset,-std::numeric_limits<double>::max()});
    std::vector<double> knots(points.size(),2.);knots.insert(knots.end(),points.size(),5.);
    SmBSplineCurve curve(3,ULONG(points.size()-1),ULONG(knots.size()-1),ULONG(points.size()-1),cp.data(),knots.data());
    curve.SetContext(&context);
    SmExtent1d interval(2,5);if(mode=="trim")interval.SetMinMax(2.5,4.5);
    SmExtent2d domain(0,0,1,1);if(mode=="subset")domain.SetMinMax(SmPoint2d(.05,.05),SmPoint2d(.95,.95));
    if(mode=="outside")domain.SetMinMax(SmPoint2d(-.1,0),SmPoint2d(1,1));
    if(mode=="fallback")domain=SmExtent2d();
    SmTArray<SmBSplineCurve*> result;SmDropCurveFail fail;
    auto bsp=static_cast<SmBSplineSurface*>(surface.get());bool before=bsp->GetOutOfBoundsEnabled()!=FALSE;
    double distance=0,deviation=0;double tolerance=1e-5*scale;
    auto status=mode=="trace" ? surface->DropAndTrimCurve(context,domain,curve,interval,tolerance,distance,deviation,result,keep,FALSE,&fail)
                             : surface->DropCurve(context,domain,curve,interval,tolerance,distance,deviation,result,keep,&fail,TRUE);
    std::cout<<"{\"status\":"<<status<<",\"distance\":"<<distance<<",\"deviation\":"<<deviation<<",\"flags\":["<<(before?"true":"false")<<','<<(bsp->GetOutOfBoundsEnabled()?"true":"false")<<"],\"failure\":["<<fail.m_sFailStatus<<','<<fail.m_lFromCall<<','<<int(fail.m_eCreatePathType)<<"],\"curves\":[";
    for(ULONG i=0;i<result.GetSize();++i){if(i)std::cout<<',';auto uv=result[i];SmTArray<double> uk;SmTArray<ULONG> mult;uv->GetKnots(uk,&mult);
        std::cout<<"{\"degree\":"<<uv->GetDegree()<<",\"dimension\":"<<uv->GetDim()<<",\"knots\":[";
        for(ULONG j=0;j<uk.GetSize();++j){if(j)std::cout<<',';std::cout<<uk[j];}std::cout<<"],\"multiplicities\":[";
        for(ULONG j=0;j<mult.GetSize();++j){if(j)std::cout<<',';std::cout<<mult[j];}std::cout<<"],\"samples\":[";
        auto ivl=uv->GetNaturalInterval();for(int j=0;j<9;++j){SmPoint3d point;uv->EvaluatePoint(ivl.Evaluate(double(j)/8.),point);if(j)std::cout<<',';std::cout<<'['<<point.x<<','<<point.y<<','<<point.z<<']';}
        std::cout<<"]}";delete uv;
    }std::cout<<"]}\n";
    return 0;
}catch(...){return 3;}}
