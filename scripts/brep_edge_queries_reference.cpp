// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Actual SDK UV periodicity/pole queries on public synthetic runtime data.
#include "SmBrep.h"
#include "SmBrepData.h"
#include "SmBSplineSurface.h"
#include "SmBSplineCurve.h"
#include "SmContext.h"
#include "SmEdge.h"
#include "SmEdgeuse.h"
#include "SmLine.h"
#include "SmLoop.h"
#include "SmLoopuse.h"
#include <nurbs.h>
#include <array>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
template<class Tag,typename Tag::type Member>struct Access{friend typename Tag::type field(Tag){return Member;}};
struct Uv {using type=SmBSplineCurve* SmEdgeuse::*;friend type field(Uv);};
template struct Access<Uv,&SmEdgeuse::m_pUVTrimCurve>;
static void require(SmStatus s){if(s!=SM_SUCCESS)throw std::runtime_error("native status "+std::to_string(s));}
static const char* boolean(bool v){return v?"true":"false";}
static SmBSplineCurve* curve(SmContext& k,const std::string& name,ULONG dim){
    if(name=="line")return new(&k)SmLine(SmPoint3d(0,0,0),SmPoint3d(1,1,0),dim,&k);
    std::vector<std::array<double,2>> pts;
    if(name=="jump")pts={{0,0},{.9,0},{1,0}};
    else if(name=="below")pts={{0,0},{.9-1e-13,0},{1,0}};
    else if(name=="jump_v")pts={{0,0},{0,.9},{0,1}};
    else if(name=="dog")pts={{0,0},{0,.2},{.5,.2},{.5,.4}};
    else if(name=="turn")pts={{0,0},{0,.2},{.5,.2}};
    else if(name=="v_bug")pts={{.5,.71},{.5,.01},{.8,.16}};
    else if(name=="turn_before")pts={{.02-1e-13,0},{.02-1e-13,.2},{.5,.2}};
    else if(name=="turn_at")pts={{.02,0},{.02,.2},{.5,.2}};
    else if(name=="turn_right")pts={{1,0},{1,.2},{.5,.2}};
    else if(name=="turn_top")pts={{.2,1},{.4,1},{.4,.5}};
    else if(name=="turn_bottom")pts={{0,0},{.2,0},{.2,.5}};
    else if(name=="dog_end")pts={{.6,.8},{.7,.8},{.8,.8},{.5,.4},{.5,.2},{0,.2},{0,0}};
    else if(name=="short")pts={{0,0},{1,1}};
    else throw std::runtime_error("unknown curve");
    const short degree=(name=="dog"||name=="dog_end")?3:1;
    std::vector<double> knots(degree+1,0.);for(size_t i=1;i<pts.size()-degree;++i)knots.push_back(double(i)/double(pts.size()-degree));knots.insert(knots.end(),degree+1,1.);
    // Two cubic Bezier segments exercise a nontrivial returned polygon index.
    if(name=="dog_end")knots={0,0,0,0,.5,.5,.5,1,1,1,1};
    std::vector<gw_CPOINT> cp;for(auto p:pts)cp.push_back({p[0],p[1],dim==2?NL_NOZ:0.,NL_NOW});
    return new(&k)SmBSplineCurve(dim,ULONG(cp.size()-1),ULONG(knots.size()-1),degree,cp.data(),knots.data());
}
static SmSurface* surface(SmContext& k,const std::string& name){
    SmPoint3d p00(0,0,0),p10(1,0,0),p01(0,1,0),p11(1,1,0);
    if(name=="u_min"){p01=p00;}
    if(name=="u_max"){p10=p11=SmPoint3d(0,0,0);p00=SmPoint3d(1,0,0);p01=SmPoint3d(1,1,0);}
    if(name=="v_min"){p10=p00;}
    if(name=="v_max"){p01=p11=SmPoint3d(0,0,0);p00=SmPoint3d(0,1,0);p10=SmPoint3d(1,1,0);}
    SmBSplineSurface* s=nullptr;require(SmBSplineSurface::CreateBilinearSurface(k,p00,p10,p01,p11,s));return s;
}
int main(int argc,char**argv){try{
    if(argc<6)throw std::runtime_error("family shape surface/flags dimension mate");
    const std::string family=argv[1],name=argv[2],kind=argv[3];ULONG dim=std::stoul(argv[4]);bool mate=std::stoi(argv[5])!=0;
    std::cout<<std::setprecision(17);SmContext k;
    if(family=="periodic"){
        std::unique_ptr<SmBSplineCurve> c(curve(k,name,dim));bool before=c->GetGwNurbPointer()!=nullptr;
        int flags=std::stoi(kind);SmExtent2d d(0,0,1,1);bool value=c->AssertNoPeriodicJump((flags&1)!=0,(flags&2)!=0,d);
        std::cout<<"{\"value\":"<<boolean(value)<<",\"index\":null,\"before_nurb\":"<<boolean(before)<<",\"after_nurb\":"<<boolean(c->GetGwNurbPointer()!=nullptr)<<"}\n";return 0;
    }
    SmBrepConstructor builder;auto b=builder.StartBrep(k);SmShell* outer=nullptr;SmShell* inner=nullptr;require(builder.StartShell(outer,inner));
    builder.StartFace(SM_OT_SAME);auto s=surface(k,kind);require(builder.SetFaceSurface(s,SmExtent2d(0,0,1,1)));auto loop=builder.StartLoop(SM_OT_SAME);
    const std::array<SmPoint2d,3> uv{SmPoint2d(.2,.2),SmPoint2d(.8,.2),SmPoint2d(.2,.8)};std::array<SmPoint3d,3> p;std::array<SmVertex*,3> v;
    for(size_t i=0;i<3;++i){require(s->EvaluatePoint(uv[i],p[i]));v[i]=builder.StartVertexOfLoop(p[i]);}
    for(size_t i=0;i<3;++i){if(!builder.StartEdge(SM_OT_SAME,nullptr,v[i],v[(i+1)%3]))throw std::runtime_error("edge");require(builder.SetEdgeCurve(new(&k)SmLine(p[i],p[(i+1)%3],3,&k),FALSE));require(builder.SetLimits(SmVertexNode(0.),SmVertexNode(1.)));require(builder.EndEdge());}
    require(builder.EndLoop());require(builder.EndFace());require(builder.EndShell(FALSE));require(builder.EndBrep());
    auto use=static_cast<SmEdgeuse*>(loop->GetLoopuse()->GetEUorVU());auto c=curve(k,name,dim);use->*field(Uv{})=c;c->SetOwner(use);bool before=c->GetGwNurbPointer()!=nullptr;SmBoolean index=-1;
    bool value=(mate?use->GetMate():use)->HasDogLegNearPole(&index);
    std::cout<<"{\"value\":"<<boolean(value)<<",\"index\":";if(value)std::cout<<index;else std::cout<<"null";
    std::cout<<",\"before_nurb\":"<<boolean(before)<<",\"after_nurb\":"<<boolean(c->GetGwNurbPointer()!=nullptr)<<"}\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
