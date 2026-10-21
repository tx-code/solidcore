// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Public synthetic curve assertions on the actual SDK owning graph.
#include "SmAssertArray.h"
#include "SmBrep.h"
#include "SmBrepData.h"
#include "SmBSplineCurve.h"
#include "SmContext.h"
#include "SmEdge.h"
#include "SmEdgeuse.h"
#include "SmLine.h"
#include <nurbs.h>
#include <array>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
template<class Tag,typename Tag::type Member> struct Access {friend typename Tag::type field(Tag){return Member;}};
struct Uv {using type=SmBSplineCurve* SmEdgeuse::*;friend type field(Uv);};
template struct Access<Uv,&SmEdgeuse::m_pUVTrimCurve>;
static void require(SmStatus s){if(s!=SM_SUCCESS)throw std::runtime_error("native status "+std::to_string(s));}
static const char* boolean(bool b){return b?"true":"false";}
static void quote(const wchar_t* s){if(!s){std::cout<<"null";return;}std::cout<<'"';for(;*s;++s){if(*s==L'"'||*s==L'\\')std::cout<<'\\'<<char(*s);else if(*s==L'\n')std::cout<<"\\n";else if(*s==L'\r')std::cout<<"\\r";else std::cout<<char(*s);}std::cout<<'"';}
static SmBSplineCurve* shape(SmContext& k,const std::string& name,ULONG dim,double scale,double offset){
    if(name=="missing_nurb")return new(&k)SmBSplineCurve(dim);
    if(name=="line"||name=="line_bad"){
        auto c=new(&k) SmLine(SmPoint3d(offset,offset,dim==2?0:offset),SmPoint3d(offset+scale,offset+scale,dim==2?0:offset),dim,&k);
        if(name=="line_bad") {auto n=c->GetOrCreateGwNurbPointer();n->pol->Pw[0].x+=scale/8.;}
        return c;
    }
    int degree=2;std::vector<std::array<double,2>> points{{0,0},{.5,.5},{1,0}};
    std::vector<double> knots{0,0,0,1,1,1};
    if(name=="linear"){degree=1;points={{0,0},{1,1}};knots={0,0,1,1};}
    if(name=="cubic"||name=="rational"){degree=3;points={{0,0},{.25,1},{.75,0},{1,1}};knots={0,0,0,0,1,1,1,1};}
    if(name=="kink"){degree=1;points={{0,0},{.5,.5},{1,0}};knots={0,0,.5,1,1};}
    if(name=="start_repeat")points={{0,0},{0,0},{1,0}};
    if(name=="end_repeat")points={{0,0},{1,0},{1,0}};
    if(name=="point")points={{0,0},{0,0},{0,0}};
    if(name=="pole"){degree=1;points={{0,0},{1,0},{1,0},{2,0}};knots={0,0,.25,.75,1,1};}
    if(name=="unclamped")knots={0,0,.125,.875,1,1};
    if(name=="mult"){points={{0,0},{.25,0},{.5,0},{.5,.5},{.75,1},{1,1}};knots={0,0,0,.5,.5,.5,1,1,1};}
    if(name=="slow")points={{0,0},{1,0},{0,0.00000001}};
    std::vector<gw_CPOINT> cp;
    for(size_t i=0;i<points.size();++i){double w=name=="rational"?double(i+1):NL_NOW;
        double x=points[i][0]*scale+offset,y=points[i][1]*scale+offset,z=dim==2?(name=="tag2_z"?double(i)*scale:NL_NOZ):offset;
        if(w!=NL_NOW){x*=w;y*=w;if(dim==3)z*=w;}
        cp.push_back({x,y,z,w});}
    return new(&k) SmBSplineCurve(dim,ULONG(cp.size()-1),ULONG(knots.size()-1),short(degree),cp.data(),knots.data());
}
int main(int argc,char**argv){try{
    if(argc<7)throw std::runtime_error("shape slot level dimension scale offset");
    const std::string name=argv[1],slot=argv[2],selection=argv[3];const ULONG dim=std::stoul(argv[4]);
    SmContext k;SmBrepConstructor builder;auto b=builder.StartBrep(k);require(builder.EndBrep());
    SmShell* shell=nullptr;SmVertex* v=nullptr;require(b->MakeShellVertexTopology(nullptr,shell,v));
    SmEdge* edge=nullptr;SmVertex* end=nullptr;require(b->MakeEdgeVertexTopology(nullptr,v,edge,end));
    auto c=shape(k,name,dim,std::stod(argv[5]),std::stod(argv[6]));
    if(slot=="uv"){auto use=edge->GetPrimaryEdgeuse();use->*field(Uv{})=c;c->SetOwner(use);}
    else edge->SetCurve(c);
    if(selection=="bad_owner")c->SetOwner(b);
    const auto before=c->GetGwNurbPointer()!=nullptr;
    if(selection=="debug_lpa"||selection=="debug_gpa") {
        std::cout<<std::setprecision(17)<<"{\"solutions\":[";bool comma=false;
        for(auto prop:{SM_CP_MINIMIZE_FIRST_DERIVATIVE,SM_CP_MINIMIZE_RADIUS_OF_CURVATURE}) {
            if(selection=="debug_lpa")for(double guess:{0.,.3,.5,.8,1.}) {
                SmBoolean found=FALSE;SmSolution sol;auto status=c->LocalPropertyAnalysis(c->GetNaturalInterval(),prop,guess,nullptr,nullptr,found,sol);
                if(comma)std::cout<<',';comma=true;std::cout<<'['<<prop<<','<<guess<<','<<status<<','<<boolean(found)<<',';
                if(status==SM_SUCCESS)std::cout<<sol.m_vStart[0]<<','<<sol.m_vStart.m_dSolutionValue;else std::cout<<"null,null";std::cout<<']';
            }else {SmSolutionArray sol;SmExtent3d box;require(c->CalculateBoundingBox(c->GetNaturalInterval(),&box));auto sep=box.GetSize().Length()/20.;auto status=c->GlobalPropertyAnalysis(c->GetNaturalInterval(),prop,nullptr,nullptr,sep,sol);
                for(ULONG i=0;i<sol.GetSize();++i){if(comma)std::cout<<',';comma=true;std::cout<<'['<<prop<<','<<status<<','<<sol[i].m_vStart[0]<<','<<sol[i].m_vStart.m_dSolutionValue<<']';}}
        }
        std::cout<<"]}\n";return 0;
    }
    SmAssertArray reports;SmTArray<ULONG> req;
    auto level=selection.starts_with("given")?SM_LEVEL_GIVEN:selection=="two"?SM_LEVEL_2:selection=="one"?SM_LEVEL_1:selection=="heal"?SM_LEVEL_HEAL:SM_LEVEL_0;
    if(selection=="given")req.Add(0);if(selection=="given_radius")req.Add(dim==2?13:14);
    auto target=selection=="no_reports"?nullptr:&reports;
    auto call=[&]{return c->AssertValid(target,level,SM_NO_WALK,level==SM_LEVEL_GIVEN?&req:nullptr);};
    const bool first=call();bool valid=first;if(selection=="repeat")valid=call();
    std::cout<<std::setprecision(17)<<"{\"curve_type\":"<<c->GetType()<<",\"valid\":"<<boolean(valid)<<",\"first\":"<<boolean(first)<<",\"before_nurb\":"<<boolean(before)<<",\"after_nurb\":"<<boolean(c->GetGwNurbPointer()!=nullptr)<<",\"last_add\":"<<boolean(reports.GetLastAdd())<<",\"reports\":[";
    const auto key=slot=="uv"?(3ULL<<62):(1ULL<<62);
    for(ULONG i=0;i<reports.GetSize();++i){if(i)std::cout<<',';auto r=reports[i];if(r->m_pOwner!=c)throw std::runtime_error("report owner");
        std::cout<<'['<<r->m_lReportingType<<','<<r->m_lListIndex<<','<<r->m_lTestIndex<<','<<r->m_eAssertType<<','<<key<<','<<r->m_lOwnerType<<',';
        quote(r->m_pName);std::cout<<',';quote(r->m_pMessage);std::cout<<','<<r->m_sTol3d<<','<<r->m_dValue<<',';quote(r->m_sOwnerTypeString);
        std::cout<<",["<<r->m_sOwnerParam.x<<','<<r->m_sOwnerParam.y<<"]]";}
    std::cout<<"]}\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
